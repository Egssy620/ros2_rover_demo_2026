#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <chrono>
#include <cmath>
#include <functional>
#include <stdexcept>
#include <string>
#include <vector>

// Master node: 各ノードの結果を受けてフェーズを進め、次の相手に依頼・中継する (計算はしない)。
// フェーズ (Phase) × 作業 (Task) の2段。フェーズ表: Obsidian Decisions/2026-10-02-master-redesign-monitor.md
// 時間切れと相手の見張り (1 秒ごと) は知らせるだけで、フェーズは変えない (止めるのは各ノード・緊急停止の側)。
//   時間切れ: 今の作業で返事を待ち始めてから決まった時間を過ぎたら、1回だけ WARN を出す (遅れて届いた返事はそのまま受け付ける)
//   見張り  : 経路計画・把持計画・機構部がいるか、自己位置が新しいかを見て、いなくなった/戻った時に1回ずつ出す
// 荷下ろし地点 (drop_xs/ys/yaws_deg) は家電の種類ごとに1か所 (並べた数 = 種類の数)。1か所だけなら毎回そこで降ろして山へ戻る
//   (2026-10-06 試験場は荷下ろし地点が1つ。そのときはログに種類を出さない)
// 山検出 (scrap_pile_skirt_node ~/enable) は「スクラップ山: 検出」の間だけ動かし、抜けたら (物品発見・物品なし) 止める。開始時にも止める
//   (2026-10-06 本人指示: RViz に山検出が出続けるので、家電認識の作業以外では止める)。
//   動かしてから pile_detect_warmup 秒待って把持計画へ頼む (把持計画は 1 s より古い点群では計算しないため)。
//   止めている間 simple_goal_follower_node は近すぎの判定をしない (ピックアップ位置への移動も)

namespace
{
// ===== フェーズ =====
enum class Phase {
    Idle,
    Pile,
    Pickup,
    Drop,
    Finished,
};

// ===== 作業 =====
enum class Task {
    Find,
    Plan,
    Drive,
    Work,
};

// ===== 届く結果 =====
enum class Result {
    Start,
    PathPlanned,
    Arrived,
    ItemFound,
    NoItem,
    Grasped,
    Missed,
    BoxFull,
    DropFound,
    Unloaded,
};

// ===== 作業名 (UI 表示用。master_ui.py の PHASES と同じ名前) =====
std::string taskName(Phase p, Task t)
{
    switch (p) {
    case Phase::Idle:     return "待機";
    case Phase::Finished: return "終了";
    case Phase::Pile:
        switch (t) {
        case Task::Plan:  return "スクラップ山: 経路生成";
        case Task::Drive: return "スクラップ山: 移動";
        case Task::Work:  return "スクラップ山: 検出";
        default:          break;
        }
        break;
    case Phase::Pickup:
        switch (t) {
        case Task::Plan:  return "ピックアップ: 経路生成";
        case Task::Drive: return "ピックアップ: 移動";
        case Task::Work:  return "ピックアップ";
        default:          break;
        }
        break;
    case Phase::Drop:
        switch (t) {
        case Task::Find:  return "荷降ろし場所探索";
        case Task::Plan:  return "荷降ろし: 経路生成";
        case Task::Drive: return "荷降ろし: 移動";
        case Task::Work:  return "荷降ろし";
        }
        break;
    }
    return "不明";
}

std::string resultName(Result r)
{
    switch (r) {
    case Result::Start:       return "開始";
    case Result::PathPlanned: return "経路生成完了";
    case Result::Arrived:     return "到着";
    case Result::ItemFound:   return "家電山発見";
    case Result::NoItem:      return "家電山なし";
    case Result::Grasped:     return "ピックアップ完了";
    case Result::Missed:      return "空振り";
    case Result::BoxFull:     return "箱満杯";
    case Result::DropFound:   return "荷降ろし場所発見";
    case Result::Unloaded:    return "荷降ろし完了";
    }
    return "不明";
}

geometry_msgs::msg::PoseStamped makePose(double x, double y, double yaw_deg)
{
    const double yaw = yaw_deg * M_PI / 180.0;
    geometry_msgs::msg::PoseStamped p;
    p.header.frame_id = "map";
    p.pose.position.x = x;
    p.pose.position.y = y;
    p.pose.orientation.z = std::sin(0.5 * yaw);
    p.pose.orientation.w = std::cos(0.5 * yaw);
    return p;
}
}  // namespace

class MasterNode : public rclcpp::Node
{
public:
    MasterNode() : Node("master_node")
    {
        pickup_end_wait_ = declare_parameter<double>("pickup_end_wait", 1.0);  // [s]
        pile_detect_warmup_ = declare_parameter<double>("pile_detect_warmup", 1.0);  // [s]

        // ===== 時間切れ [s] =====
        // 経路生成: 直線の代役も林研版も、目標を受けたらすぐ planned を返すので短くてよい
        plan_timeout_ = declare_parameter<double>("plan_timeout", 10.0);
        // 移動: 目標までの直線距離 / 想定速度 × 余裕倍率。想定速度 = 簡易移動の最大並進速度 (prototype の既定 0.10 m/s)、
        //       余裕倍率 = 回り道・向きを変える時間・加減速の分 (仮値)
        drive_speed_ = declare_parameter<double>("drive_speed", 0.10);    // [m/s]
        drive_margin_ = declare_parameter<double>("drive_margin", 2.0);
        // 山の検出: 把持計画はサービスの中で1回計算して返すだけ
        check_timeout_ = declare_parameter<double>("check_timeout", 5.0);
        // 機構部 (つかむ・下ろす): 実機の動作時間が未定のため仮値
        mechanism_timeout_ = declare_parameter<double>("mechanism_timeout", 60.0);
        // 見張り: 自己位置がこれより古ければ「来ていない」(localizer は検出のたび ~10 Hz、UI の POSE_TIMEOUT と同じ)
        pose_timeout_ = declare_parameter<double>("pose_timeout", 1.0);
        // 見張り: 起動直後は相手がまだ立ち上がっていないことがあるので、この時間は見ない
        watch_start_delay_ = declare_parameter<double>("watch_start_delay", 5.0);

        // ===== 代役のための固定値 =====
        const auto pile = declare_parameter<std::vector<double>>("pile_front", {0.0, 0.0, 0.0});  // x, y, yaw_deg
        const auto xs = declare_parameter<std::vector<double>>("drop_xs", {0.0, 0.0, 0.0});
        const auto ys = declare_parameter<std::vector<double>>("drop_ys", {0.0, 0.0, 0.0});
        const auto yaws = declare_parameter<std::vector<double>>("drop_yaws_deg", {0.0, 0.0, 0.0});
        if (pile.size() != 3) throw std::runtime_error("pile_front は [x, y, yaw_deg] の3つ");
        if (xs.empty() || xs.size() != ys.size() || xs.size() != yaws.size())
            throw std::runtime_error("drop_xs / drop_ys / drop_yaws_deg は同じ数 (家電の種類の数, 1 以上) だけ並べる");
        pile_front_ = makePose(pile[0], pile[1], pile[2]);
        for (size_t i = 0; i < xs.size(); ++i) drops_.push_back(makePose(xs[i], ys[i], yaws[i]));
        type_count_ = static_cast<int>(drops_.size());

        // ===== 渡す =====
        pub_goal_ = create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);
        pub_mech_cmd_ = create_publisher<std_msgs::msg::String>("/mechanism/command", 10);
        pub_mech_grasp_ = create_publisher<geometry_msgs::msg::PointStamped>("/mechanism/grasp_point", 10);
        pub_phase_ = create_publisher<std_msgs::msg::String>("~/phase", rclcpp::QoS(1).transient_local());
        planner_next_ = create_client<std_srvs::srv::Trigger>("/pickup_planner_node/next");
        pile_enable_ = create_client<std_srvs::srv::SetBool>("/scrap_pile_skirt_node/enable");

        // ===== 受け取る =====
        srv_start_ = create_service<std_srvs::srv::Trigger>(
            "~/start", [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                              std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
                res->success = (phase_ == Phase::Idle);
                res->message = res->success ? "開始" : "待機中でないので開始できない (今: " + taskName(phase_, task_) + ")";
                if (res->success) onResult(Result::Start);
            });
        sub_nav_ = create_subscription<std_msgs::msg::String>(
            "/navigation/result", 10, [this](std_msgs::msg::String::SharedPtr m) {
                if (m->data == "planned") onResult(Result::PathPlanned);
                else if (m->data == "arrived") onResult(Result::Arrived);
                else if (m->data == "blocked") RCLCPP_WARN(get_logger(), "障害物に近すぎて、目標まで進めずに止まりました（%s）", taskName(phase_, task_).c_str());
                else RCLCPP_WARN(get_logger(), "経路計画から知らない返事「%s」が届きました。無視します", m->data.c_str());
            });
        sub_grasp_ = create_subscription<geometry_msgs::msg::PointStamped>(
            "/pickup_planner_node/grasp_point", 10, [this](geometry_msgs::msg::PointStamped::SharedPtr m) { grasp_point_ = *m; });
        sub_pickup_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/pickup_pose", 10, [this](geometry_msgs::msg::PoseStamped::SharedPtr m) {
                pickup_pose_ = *m;
                onResult(Result::ItemFound);
            });
        sub_pose_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/robot_pose", 10, [this](geometry_msgs::msg::PoseStamped::SharedPtr m) {
                robot_pose_ = *m;
                pose_rx_ = now();
                have_pose_ = true;
            });
        sub_mech_ = create_subscription<std_msgs::msg::String>(
            "/mechanism/result", 10, [this](std_msgs::msg::String::SharedPtr m) {
                if (m->data == "grasped") onResult(Result::Grasped);
                else if (m->data == "missed") onResult(Result::Missed);
                else if (m->data == "full") onResult(Result::BoxFull);
                else if (m->data == "unloaded") onResult(Result::Unloaded);
                else RCLCPP_WARN(get_logger(), "機構部から知らない返事「%s」が届きました。無視します", m->data.c_str());
            });

        started_ = now();
        task_started_ = now();
        watch_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {
            checkTimeout();
            watchPartners();
        });

        publishPhase();
        RCLCPP_INFO(get_logger(), "準備できました。UI の「開始」を押すと始まります（端末からは ros2 service call /master_node/start std_srvs/srv/Trigger）");
    }

private:
    // ===== 結果の振り分け =====
    void onResult(Result r)
    {
        if (waiting()) {
            RCLCPP_WARN(get_logger(), "（待ち時間中なので「%s」は無視します）", resultName(r).c_str());
            return;
        }
        if (handle(r)) return;
        RCLCPP_WARN(get_logger(), "「%s」が届きましたが、今（%s）は待っていないので無視します",
                    resultName(r).c_str(), taskName(phase_, task_).c_str());
    }

    bool handle(Result r)
    {
        if (phase_ == Phase::Idle) {
            if (r != Result::Start) return false;
            say("開始しました。山の前", pile_front_, "へ向かいます");
            setPileDetection(false);
            startMove(Phase::Pile, pile_front_);
            return true;
        }
        if (phase_ == Phase::Finished) return false;

        switch (task_) {
        case Task::Find:  return onFind(r);
        case Task::Plan:  return onPlan(r);
        case Task::Drive: return onDrive(r);
        case Task::Work:  return onWork(r);
        }
        return false;
    }

    // ===== 探索 =====
    bool onFind(Result r)
    {
        if (phase_ != Phase::Drop || r != Result::DropFound) return false;
        say(dropName(), drops_[unloaded_count_], "へ向かいます");
        startMove(Phase::Drop, drops_[unloaded_count_]);
        return true;
    }

    // ===== 経路生成 =====
    bool onPlan(Result r)
    {
        if (r != Result::PathPlanned) return false;
        RCLCPP_INFO(get_logger(), "経路ができました。%sへ移動します", destName().c_str());
        changePhase(phase_, Task::Drive);
        return true;
    }

    // ===== 移動 =====
    bool onDrive(Result r)
    {
        if (r != Result::Arrived) return false;
        switch (phase_) {
        case Phase::Pile:
            RCLCPP_INFO(get_logger(), "山の前に着きました。山につかめる物があるか調べます");
            startCheckPile();
            return true;
        case Phase::Pickup:
            RCLCPP_INFO(get_logger(), "つかむ位置に着きました。機構部に、高さ %.2f m の点をつかむよう頼みます",
                        grasp_point_.point.z);
            startGrasp();
            return true;
        case Phase::Drop:
            RCLCPP_INFO(get_logger(), "%sに着きました。機構部に、%sを下ろすよう頼みます",
                        dropNameInText().c_str(), kindName().c_str());
            startUnload();
            return true;
        default:
            return false;
        }
    }

    // ===== 現地での作業 =====
    bool onWork(Result r)
    {
        switch (phase_) {
        case Phase::Pile:   return onCheckPileDone(r);
        case Phase::Pickup: return onGraspDone(r);
        case Phase::Drop:   return onUnloadDone(r);
        default:           return false;
        }
    }

    // ----- 山の検出 -----
    bool onCheckPileDone(Result r)
    {
        if (r == Result::ItemFound) {
            setPileDetection(false);
            say("つかめる物がありました。つかむ位置", pickup_pose_, "へ向かいます");
            startMove(Phase::Pickup, pickup_pose_);
            return true;
        }
        if (r == Result::NoItem) {
            RCLCPP_INFO(get_logger(), "山にもう物がありません");
            setPileDetection(false);
            pile_finished_ = true;
            startFindDrop();
            return true;
        }
        return false;
    }

    // ----- 把持 -----
    bool onGraspDone(Result r)
    {
        if (r == Result::Grasped || r == Result::Missed) {
            RCLCPP_INFO(get_logger(), "%s。%.1f 秒待ってから、山をもう一度調べます",
                        r == Result::Grasped ? "つかみました" : "つかみ損ねました", pickup_end_wait_);
            waitThen(pickup_end_wait_, [this]() { startCheckPile(); });
            return true;
        }
        if (r == Result::BoxFull) {
            RCLCPP_INFO(get_logger(), "箱がいっぱいです。%.1f 秒待ってから、荷下ろしへ向かいます", pickup_end_wait_);
            waitThen(pickup_end_wait_, [this]() { startFindDrop(); });
            return true;
        }
        return false;
    }

    // ----- 荷下ろし -----
    bool onUnloadDone(Result r)
    {
        if (r != Result::Unloaded) return false;
        const std::string kind = kindName();
        ++unloaded_count_;
        if (unloaded_count_ < type_count_) {
            RCLCPP_INFO(get_logger(), "%sを下ろしました", kind.c_str());
            startFindDrop();
        } else if (pile_finished_) {
            RCLCPP_INFO(get_logger(), "%sを下ろしました。山も空なので、作業はすべて終わりです", kind.c_str());
            changePhase(Phase::Finished, Task::Plan);
        } else {
            unloaded_count_ = 0;
            say(type_count_ > 1 ? "全種類を下ろしました。山の前" : "下ろしました。山の前", pile_front_, "へ戻ります");
            startMove(Phase::Pile, pile_front_);
        }
        return true;
    }

    // ===== 動作 (フェーズと作業を先に切り替えてから依頼する) =====
    void startMove(Phase p, geometry_msgs::msg::PoseStamped goal)
    {
        changePhase(p, Task::Plan);
        goal.header.stamp = now();
        goal_ = goal;
        pub_goal_->publish(goal);
    }

    // 山検出を動かし、点群がそろうのを待ってから把持計画へ頼む
    void startCheckPile()
    {
        changePhase(Phase::Pile, Task::Work);
        setPileDetection(true);
        waitThen(pile_detect_warmup_, [this]() { askPlanner(); });
    }

    // 物があれば /pickup_pose が届く (→ ItemFound)、なければ応答が「完了: ...」(→ NoItem)
    void askPlanner()
    {
        if (!planner_next_->service_is_ready()) {
            RCLCPP_ERROR(get_logger(), "把持計画ノードに頼めません。pickup_planner_node が起動しているか確認してください");
            return;
        }
        planner_next_->async_send_request(
            std::make_shared<std_srvs::srv::Trigger::Request>(),
            [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture f) {
                const auto res = f.get();
                if (!res->success) {
                    RCLCPP_ERROR(get_logger(), "把持計画が、つかむ位置を計算できませんでした（理由: %s）", res->message.c_str());
                    return;
                }
                if (res->message.rfind("完了", 0) == 0) onResult(Result::NoItem);
            });
    }

    void setPileDetection(bool on)
    {
        if (!pile_enable_->service_is_ready()) {
            RCLCPP_ERROR(get_logger(), "山検出を%sられません。scrap_pile_skirt_node が起動しているか確認してください",
                         on ? "動かせ" : "止め");
            return;
        }
        auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
        req->data = on;
        pile_enable_->async_send_request(req);
    }

    void startGrasp()
    {
        changePhase(Phase::Pickup, Task::Work);
        grasp_point_.header.stamp = now();
        pub_mech_grasp_->publish(grasp_point_);
        std_msgs::msg::String cmd;
        cmd.data = "grasp";
        pub_mech_cmd_->publish(cmd);
    }

    // 代役: 荷下ろし地点はパラメータ固定なので、すぐ見つかったことにする
    void startFindDrop()
    {
        changePhase(Phase::Drop, Task::Find);
        onResult(Result::DropFound);
    }

    void startUnload()
    {
        changePhase(Phase::Drop, Task::Work);
        std_msgs::msg::String cmd;
        cmd.data = "unload " + std::to_string(unloaded_count_ + 1);
        pub_mech_cmd_->publish(cmd);
    }

    // ===== 補助 =====
    std::string destName() const
    {
        switch (phase_) {
        case Phase::Pile:   return "山の前";
        case Phase::Pickup: return "つかむ位置";
        case Phase::Drop:   return dropNameInText();
        default:           return "";
        }
    }

    // 荷下ろし地点が種類ごとにあるときだけ番号・種類を付ける
    std::string dropName() const
    {
        return type_count_ > 1 ? "荷下ろし場所 " + std::to_string(unloaded_count_ + 1) : "荷下ろし場所";
    }

    // 文の中に入れる用 (番号で終わるときは後ろを1字空ける)
    std::string dropNameInText() const { return type_count_ > 1 ? dropName() + " " : dropName(); }

    std::string kindName() const
    {
        return type_count_ > 1 ? "種類 " + std::to_string(unloaded_count_ + 1) + " の家電" : "家電";
    }

    void say(const std::string &head, const geometry_msgs::msg::PoseStamped &p, const std::string &tail)
    {
        RCLCPP_INFO(get_logger(), "%s (%.1f, %.1f) %s", head.c_str(), p.pose.position.x, p.pose.position.y, tail.c_str());
    }

    // ===== 時間切れ =====
    // 今の作業で待てる時間 [s] と、返事をくれる相手。待つものが無いときは 0
    double timeoutFor(std::string &who) const
    {
        if (phase_ == Phase::Idle || phase_ == Phase::Finished) return 0.0;
        switch (task_) {
        case Task::Find:
            return 0.0;  // 代役: すぐ見つかる
        case Task::Plan:
            who = "経路計画";
            return plan_timeout_;
        case Task::Drive:
            who = "経路計画";
            return drive_limit_;
        case Task::Work:
            who = phase_ == Phase::Pile ? "把持計画" : "機構部";
            return phase_ == Phase::Pile ? check_timeout_ : mechanism_timeout_;
        }
        return 0.0;
    }

    void checkTimeout()
    {
        if (timeout_reported_ || waiting()) return;
        std::string who;
        const double limit = timeoutFor(who);
        if (limit <= 0.0) return;
        const double elapsed = (now() - task_started_).seconds();
        if (elapsed < limit) return;
        timeout_reported_ = true;
        RCLCPP_WARN(get_logger(), "%sの返事が %.0f 秒ありません（%s）。%sが動いているか確かめてください。このまま待ちます",
                    who.c_str(), elapsed, taskName(phase_, task_).c_str(), who.c_str());
    }

    // 移動の待ち時間 = 今の位置から目標までの直線距離 / 想定速度 × 余裕倍率。自己位置がまだ無ければ見ない (0)
    double driveLimit() const
    {
        if (!have_pose_ || drive_speed_ <= 0.0) return 0.0;
        const double d = std::hypot(goal_.pose.position.x - robot_pose_.pose.position.x,
                                    goal_.pose.position.y - robot_pose_.pose.position.y);
        return d / drive_speed_ * drive_margin_;
    }

    // ===== 相手の見張り =====
    // 経路計画・機構部 = 結果のトピックを出す側がいるか、把持計画 = サービスがあるか、自己位置 = 新しいか
    void watchPartners()
    {
        if ((now() - started_).seconds() < watch_start_delay_) return;
        const bool pose_ok = have_pose_ && (now() - pose_rx_).seconds() < pose_timeout_;
        report(count_publishers("/navigation/result") > 0, nav_ok_, "経路計画が戻りました",
               "経路計画が見つかりません。経路計画のノードを起動しているか確かめてください");
        report(planner_next_->service_is_ready(), planner_ok_, "把持計画が戻りました",
               "把持計画が見つかりません。pickup_planner_node を起動しているか確かめてください");
        report(count_publishers("/mechanism/result") > 0, mech_ok_, "機構部が戻りました",
               "機構部が見つかりません。機構部のノードを起動しているか確かめてください");
        report(pose_ok, pose_ok_, "自己位置がまた届くようになりました",
               "自己位置が届いていません。ランドマークが見えているか、自己位置のノードが動いているか確かめてください");
    }

    // 状態が変わった時だけ 1 回出す
    void report(bool ok, bool &last, const std::string &back, const std::string &lost)
    {
        if (ok == last) return;
        last = ok;
        if (ok) RCLCPP_INFO(get_logger(), "%s", back.c_str());
        else RCLCPP_WARN(get_logger(), "%s", lost.c_str());
    }

    void waitThen(double sec, std::function<void()> next)
    {
        wait_timer_ = create_wall_timer(std::chrono::duration<double>(sec), [this, next]() {
            wait_timer_->cancel();
            next();
        });
    }

    bool waiting() const { return wait_timer_ && !wait_timer_->is_canceled(); }

    void changePhase(Phase p, Task t)
    {
        phase_ = p;
        task_ = t;
        task_started_ = now();
        timeout_reported_ = false;
        if (t == Task::Drive) drive_limit_ = driveLimit();
        publishPhase();
    }

    void publishPhase()
    {
        std_msgs::msg::String m;
        m.data = taskName(phase_, task_);
        pub_phase_->publish(m);
    }

    // ===== 今のフェーズと作業 =====
    Phase phase_ = Phase::Idle;
    Task task_ = Task::Plan;

    // ===== 覚えておく情報 =====
    int unloaded_count_ = 0;
    bool pile_finished_ = false;

    // ===== 中継する値 =====
    geometry_msgs::msg::PoseStamped pickup_pose_;
    geometry_msgs::msg::PointStamped grasp_point_;

    // ===== 決まっている値 =====
    double pickup_end_wait_ = 1.0;
    double pile_detect_warmup_ = 1.0;  // 山検出を動かしてから把持計画へ頼むまで待つ時間 [s]
    double plan_timeout_ = 10.0, drive_speed_ = 0.10, drive_margin_ = 2.0, check_timeout_ = 5.0, mechanism_timeout_ = 60.0;
    double pose_timeout_ = 1.0, watch_start_delay_ = 5.0;

    // ===== 時間切れ・見張り =====
    rclcpp::Time started_, task_started_, pose_rx_;
    bool timeout_reported_ = false;
    double drive_limit_ = 0.0;
    geometry_msgs::msg::PoseStamped goal_, robot_pose_;
    bool have_pose_ = false;
    bool nav_ok_ = true, planner_ok_ = true, mech_ok_ = true, pose_ok_ = true;  // 見張りの前回の状態 (最初は「いる」扱い)
    int type_count_ = 1;  // 家電の種類の数 (= 荷下ろし地点の数)
    geometry_msgs::msg::PoseStamped pile_front_;
    std::vector<geometry_msgs::msg::PoseStamped> drops_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_goal_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_mech_cmd_, pub_phase_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_mech_grasp_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr planner_next_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr pile_enable_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_start_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_nav_, sub_mech_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pickup_, sub_pose_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_grasp_;
    rclcpp::TimerBase::SharedPtr wait_timer_, watch_timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MasterNode>());
    rclcpp::shutdown();
    return 0;
}

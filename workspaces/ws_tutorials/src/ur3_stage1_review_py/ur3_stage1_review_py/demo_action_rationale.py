"""
验收点2：Action vs Topic/Service 适配性对比演示节点

通过结构化日志解释：为什么轨迹执行必须使用 Action 而非 Topic/Service。
不依赖运行中的机械臂，纯逻辑演示。
"""

import rclpy
from rclpy.node import Node


class ActionRationaleDemoNode(Node):
    def __init__(self):
        super().__init__("demo_action_rationale_node")
        self._explain()

    def _explain(self):
        log = self.get_logger()

        log.info("=" * 60)
        log.info("  验收点 2：Action 适配性分析 — 轨迹执行为何必须用 Action")
        log.info("=" * 60)

        # ── Topic ──────────────────────────────────────────────
        log.info("")
        log.info("  【方案 A】Topic（发布/订阅）")
        log.info("  ┌─ 发送方式 : publisher.publish(goal)")
        log.info("  ├─ 反馈机制 : 无，发完即忘")
        log.info("  ├─ 结果确认 : 无法知道轨迹是否执行完毕")
        log.info("  ├─ 中途取消 : 不支持")
        log.info("  └─ 结论     : ❌ 无法满足「等待完成 + 取消」需求")

        # ── Service ────────────────────────────────────────────
        log.info("")
        log.info("  【方案 B】Service（请求/响应）")
        log.info("  ┌─ 发送方式 : client.call(request)")
        log.info("  ├─ 反馈机制 : 仅有一次 response，无中间反馈")
        log.info("  ├─ 结果确认 : response 到达即为结果，但调用期间阻塞")
        log.info("  ├─ 中途取消 : 不支持")
        log.info("  ├─ 耗时场景 : 轨迹执行通常 1-30 秒，长期阻塞不可接受")
        log.info("  └─ 结论     : ❌ 无法做到「执行中持续反馈 + 随时取消」")

        # ── Action ─────────────────────────────────────────────
        log.info("")
        log.info("  【方案 C】Action（目标/反馈/结果）✅")
        log.info("  ┌─ 发送方式 : client.send_goal_async(goal)")
        log.info("  ├─ 反馈机制 : 执行期间持续推送 feedback（当前关节位置等）")
        log.info("  ├─ 结果确认 : 轨迹完成后推送 result（成功/失败/取消）")
        log.info("  ├─ 中途取消 : client.cancel_goal_async() 可随时中止")
        log.info("  ├─ 非阻塞   : 异步设计，调用方不会被卡住")
        log.info("  └─ 结论     : ✅ 完全满足轨迹执行的三大需求")

        # ── 总结对比表 ─────────────────────────────────────────
        log.info("")
        log.info("  ┌──────────────┬────────┬─────────┬────────┐")
        log.info("  │ 能力         │ Topic  │ Service │ Action │")
        log.info("  ├──────────────┼────────┼─────────┼────────┤")
        log.info("  │ 异步非阻塞   │  ✅   │   ❌   │  ✅   │")
        log.info("  │ 中间反馈     │  ❌   │   ❌   │  ✅   │")
        log.info("  │ 完成确认     │  ❌   │   ✅   │  ✅   │")
        log.info("  │ 随时取消     │  ❌   │   ❌   │  ✅   │")
        log.info("  └──────────────┴────────┴─────────┴────────┘")

        log.info("")
        log.info("  [结论] FollowJointTrajectory 使用 Action 接口，")
        log.info("         正是因为它同时需要：异步执行 + 实时反馈 + 结果确认 + 可取消。")
        log.info("=" * 60)


def main(args=None):
    rclpy.init(args=args)
    node = ActionRationaleDemoNode()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

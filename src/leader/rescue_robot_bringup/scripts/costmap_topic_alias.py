#!/usr/bin/env python3
"""Expose a complete Nav2 local grid at the legacy /costmap/costmap name."""

import rclpy
from map_msgs.msg import OccupancyGridUpdate
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class CostmapTopicAlias(Node):
    def __init__(self):
        super().__init__("costmap_topic_alias")
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.grid = None
        self.publisher = self.create_publisher(OccupancyGrid, "/costmap/costmap", qos)
        self.full_subscription = self.create_subscription(
            OccupancyGrid,
            "/local_costmap/costmap",
            self.on_full_grid,
            qos,
        )
        self.update_subscription = self.create_subscription(
            OccupancyGridUpdate,
            "/local_costmap/costmap_updates",
            self.on_grid_update,
            qos,
        )
        self.timer = self.create_timer(0.5, self.publish_grid)

    def on_full_grid(self, grid):
        self.grid = grid

    def on_grid_update(self, update):
        if self.grid is None:
            return
        width = self.grid.info.width
        height = self.grid.info.height
        if (update.x + update.width > width or update.y + update.height > height
                or len(update.data) != update.width * update.height):
            self.get_logger().warning("Ignoring an out-of-bounds costmap update")
            return
        for row in range(update.height):
            start = (update.y + row) * width + update.x
            source = row * update.width
            self.grid.data[start:start + update.width] = update.data[source:source + update.width]
        self.grid.header.stamp = update.header.stamp

    def publish_grid(self):
        if self.grid is not None:
            self.publisher.publish(self.grid)


def main():
    rclpy.init()
    node = CostmapTopicAlias()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

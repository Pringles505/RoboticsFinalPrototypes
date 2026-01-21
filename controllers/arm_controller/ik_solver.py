# Inverse Kinematics Solver for 3-link robot arm
import math
from dataclasses import dataclass


@dataclass
class IKResult:
    success: bool
    shoulder: float = 0.0
    elbow: float = 0.0
    wrist: float = 0.0
    message: str = ""
    clamped: bool = False


class IKSolver:
    def __init__(self, link_lengths: list[float], base_height: float = 0.0):
        self.L1 = link_lengths[0]  # 0.4m
        self.L2 = link_lengths[1]  # 0.3m
        self.L3 = link_lengths[2]  # 0.2m
        self.base_height = base_height
        self.total_reach = self.L1 + self.L2 + self.L3


    # Solve function original FABRIK algorithm replaced with 2-link law of cosines. 
    # Por alguna razón law of cosines funciona mejor con la council de ias que FABRIK nideapq
    def solve(self, x: float, y: float, z: float) -> IKResult:
        # Shoulder yaw (rotation around Z to point toward target)
        r_xy = math.sqrt(x*x + y*y)
        if r_xy < 0.001:
            shoulder_yaw = 0.0
        else:
            shoulder_yaw = math.atan2(y, x)

        # Height relative to shoulder pivot
        z_rel = z - self.base_height

        # We want end effector horizontal, so link3 points in +X direction
        wr = r_xy - self.L3
        wz = z_rel

        # Distance from shoulder to wrist point
        d = math.sqrt(wr*wr + wz*wz)

        # Check reachability for 2-link arm (L1, L2) to reach wrist point
        L12_max = self.L1 + self.L2
        L12_min = abs(self.L1 - self.L2)

        clamped = False
        if d > L12_max * 0.98:
            # Too far - scale down
            scale = (L12_max * 0.98) / d
            wr *= scale
            wz *= scale
            d = L12_max * 0.98
            clamped = True
        elif d < L12_min * 1.05:
            # Too close - scale up
            if d > 0.001:
                scale = (L12_min * 1.05) / d
                wr *= scale
                wz *= scale
                d = L12_min * 1.05
            else:
                wr = L12_min * 1.05
                wz = 0
                d = L12_min * 1.05
            clamped = True

        # 2-link IK using law of cosines better than FABRIK since we want specific angles and 1 pass

        # Interior angle at elbow
        cos_alpha = (self.L1*self.L1 + self.L2*self.L2 - d*d) / (2 * self.L1 * self.L2)
        cos_alpha = max(-1, min(1, cos_alpha))
        alpha = math.acos(cos_alpha)  # Interior angle at elbow

        # Angle at shoulder (in the triangle)
        cos_gamma = (self.L1*self.L1 + d*d - self.L2*self.L2) / (2 * self.L1 * d)
        cos_gamma = max(-1, min(1, cos_gamma))
        gamma = math.acos(cos_gamma)

        # Angle from horizontal to wrist point
        beta = math.atan2(wz, wr)

        # Elbow-up solution:
        theta1 = beta + gamma

        theta2 = alpha - math.pi

        theta3 = -(theta1 + theta2)

        # Negate elbow and wrist because Webots Y-axis rotation:
        return IKResult(
            success=True,
            shoulder=shoulder_yaw,
            elbow=-theta1,
            wrist=-theta2,
            message="OK" + (" (clamped)" if clamped else ""),
            clamped=clamped
        )

    def forward_kinematics(self, shoulder: float, elbow: float, wrist: float) -> tuple[float, float, float]:
        # Negate back since motor angles are inverted
        theta1 = -elbow  
        theta2 = -wrist 
        theta3 = -(theta1 + theta2)  

        # Link1 end point (in arm plane: r, z)
        r1 = self.L1 * math.cos(theta1)
        z1 = self.L1 * math.sin(theta1)

        # Link2 end point (wrist)
        link2_angle = theta1 + theta2  
        r2 = r1 + self.L2 * math.cos(link2_angle)
        z2 = z1 + self.L2 * math.sin(link2_angle)

        # End effector (link3 is horizontal)
        ee_angle = link2_angle + theta3  
        r_ee = r2 + self.L3 * math.cos(ee_angle)
        z_ee = z2 + self.L3 * math.sin(ee_angle)

        # Convert to world coordinates using shoulder yaw
        x = r_ee * math.cos(shoulder)
        y = r_ee * math.sin(shoulder)
        z = z_ee + self.base_height

        return (x, y, z)


if __name__ == "__main__":
    solver = IKSolver([0.4, 0.3, 0.2], base_height=0.125)

    print("IK Solver Test")
    print("=" * 50)
    print(f"Links: {solver.L1}, {solver.L2}, {solver.L3}")
    print(f"Base height: {solver.base_height}")
    print(f"Total reach: {solver.total_reach}")
    print()

    # Test rest position
    rest = solver.forward_kinematics(0, 0, 0)
    print(f"At (0,0,0): EE at ({rest[0]:.3f}, {rest[1]:.3f}, {rest[2]:.3f})")
    print(f"Expected: ({solver.total_reach:.3f}, 0, {solver.base_height:.3f})")
    print()

    # Test various targets
    targets = [
        (0.5, 0.0, 0.25),
        (0.5, 0.1, 0.20),
        (0.6, 0.0, 0.30),
        (0.4, 0.0, 0.25),
        (0.5, -0.1, 0.20),
    ]

    for target in targets:
        result = solver.solve(*target)
        actual = solver.forward_kinematics(result.shoulder, result.elbow, result.wrist)
        error = math.sqrt(sum((a-t)**2 for a, t in zip(actual, target)))

        print(f"Target: ({target[0]:.2f}, {target[1]:.2f}, {target[2]:.2f})")
        print(f"  Angles: sh={math.degrees(result.shoulder):.1f}°, "
              f"el={math.degrees(result.elbow):.1f}°, "
              f"wr={math.degrees(result.wrist):.1f}°")
        print(f"  Actual: ({actual[0]:.3f}, {actual[1]:.3f}, {actual[2]:.3f})")
        print(f"  Error: {error*1000:.1f}mm {result.message}")
        print()

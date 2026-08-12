#!/usr/bin/env python3
"""Add sensor frames (livox_frame, camera_link) to dog.urdf"""

with open("dog.urdf", "r") as f:
    content = f.read()

sensors = '''
  <!-- ====== Sensors ====== -->
  <!-- Livox Mid-360 LiDAR: mounted on head bump, X:0.30 Z:0.15 -->
  <link name="livox_frame">
    <inertial>
      <mass value="0.005"/>
      <inertia ixx="0.000001" ixy="0" ixz="0" iyy="0.000001" iyz="0" izz="0.000001"/>
    </inertial>
  </link>
  <joint name="livox_joint" type="fixed">
    <parent link="base_link"/>
    <child link="livox_frame"/>
    <origin xyz="0.30 0 0.15" rpy="0 0 0"/>
  </joint>

  <!-- Intel D435i: flat forward-facing camera, X:0.35 Z:0.08, Pitch=0 -->
  <link name="camera_link">
    <inertial>
      <mass value="0.005"/>
      <inertia ixx="0.000001" ixy="0" ixz="0" iyy="0.000001" iyz="0" izz="0.000001"/>
    </inertial>
  </link>
  <joint name="camera_joint" type="fixed">
    <parent link="base_link"/>
    <child link="camera_link"/>
    <origin xyz="0.35 0 0.08" rpy="0 0 0"/>
  </joint>

</robot>'''

content = content.replace("</robot>", sensors)
with open("dog.urdf", "w") as f:
    f.write(content)
print("SENSOR FRAMES ADDED to dog.urdf")

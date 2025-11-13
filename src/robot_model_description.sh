URDF="$(rospack find yeah)/urdf/yeah.urdf"
{ echo "robot_description: |"; sed 's/^/  /' "$URDF"; } > /tmp/robot_description.yaml
rosparam load /tmp/robot_description.yaml
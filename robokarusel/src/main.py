import rospy
from robokarusel import srv

rospy.init_node('main')

get_distance = rospy.ServiceProxy('get_distance', srv.GetDistance)
follow_line = rospy.ServiceProxy('following_line', srv.FollowLine)
current_object = rospy.ServiceProxy('search_object', srv.SearchObject)
is_pressed = rospy.ServiceProxy('is_pressed', srv.ButtonState)
control_servo = rospy.ServiceProxy('control_servo', srv.ControlServo)

target_color = 'red'
sum = 0;

print("ready")
while True:
    while not is_pressed(True).state:
        continue
    print("go")

    for i in range(9):
        object = current_object(True).object
        current_color = current_object(True).color

        if object == 'cyl':
            control_servo('right')
            dist = 10
        elif current_color != target_color:
            control_servo('left')
            dist = 19
        else:
            control_servo('center')
            dist = 19

        time.sleep(.2)
        follow_line(kp=-2, ki=0, kd=0, speed=40)

        while get_distance(True).distance > dist:
            continue

        follow_line(speed=0)

        if object == 'cyl':
            control_servo('left')
        elif current_color != target_color:
            control_servo('right')
        else:
            control_servo('center')



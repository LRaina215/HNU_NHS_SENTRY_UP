# Robomaster target name
# [options:] [Armour|lightstrip|Rune|Mineral]
string class_id

geometry_msgs/Polygon pose
uint16 img_width
uint16 img_height
int32 center_dist

# Robomaster target type
# [options:] 
# 0-7                                , if target is armour
# armour | center                    , if target is Rune
# gold   | silver                    , if target is Mineral
string type

# traking id
string id
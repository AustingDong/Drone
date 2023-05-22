def following(trackbox_center, trackbox_height, img_center):
    sgn = ''

    disturb = 50

    tb_x = trackbox_center[0]
    tb_y = trackbox_center[1]

    ct_x = img_center[0]
    ct_y = img_center[1]

    if (tb_x == 0 and tb_y == 0):
        return sgn

    if(tb_x+disturb < ct_x): sgn = 'd'
    elif(tb_x-disturb > ct_x): sgn = 'a'

    # if(tb_y < ct_y): sgn = 'DOWN'
    # elif(tb_y > ct_y): sgn = 'UP'


    return sgn
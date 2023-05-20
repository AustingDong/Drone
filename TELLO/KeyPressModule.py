import pygame



def init():
    pygame.init()
    win = pygame.display.set_mode((400,400))
    text = [
        'w -- forward',
        's -- backward',
        'a -- shift left',
        'd -- shift right',

        'arrow-up -- higher',
        'arrow-down -- lower',
        'arrow-left -- left yaw',
        'arrow-right -- right yaw',

        'f -- take off',
        'g -- touch down',
        'q -- take photo',
        'r -- recognize mode',
        't -- track mode'
    ]
        
    
    for i in range(len(text)):
        render(text[i], color=(0,255,0), win=win,pos=(20, 20+i*30))


def getKey(keyName):
    ans = False
    for eve in pygame.event.get(): pass
    keyInput = pygame.key.get_pressed()
    myKey = getattr(pygame,'K_{}'.format(keyName))

    if keyInput[myKey]:
        ans = True

    pygame.display.update()
    return ans


def render(text, color, win, pos, background=None):
    font =  pygame.font.SysFont('microsoft Yahei',30)
    content = font.render(text, False, color)
    win.blit(content,pos)
    


def main():
    if getKey("LEFT"):
        print('left')

    


if __name__ == '__main__':
    init()
    while True:
        main()
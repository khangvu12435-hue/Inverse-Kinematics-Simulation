import pygame
import numpy as np
import math
import random

screen = pygame.display.set_mode((800,500))
pygame.time.Clock().tick(60)

jointRadius = 10
hitboxSize = 10
class KinematicPair:
    def __init__(self,pos,L,globAngle):
        self.x,self.y = pos
        self.length = L
        self.globAngle = globAngle
        self.hitbox = pygame.Rect(self.x-hitboxSize/2,self.y-hitboxSize/2,hitboxSize,hitboxSize)
        self.leftClicked = False
        self.rightClicked = False
    def getEndPoint(self):
        return (self.x+self.length*math.cos(self.globAngle),self.y+self.length*math.sin(self.globAngle))
    def draw(self):
        #pygame.draw.rect(screen,(0,0,0),self.hitbox)
        pygame.draw.circle(screen,(255,0,0),(self.x,self.y),jointRadius)
        pygame.draw.line(screen,(0,0,0),(self.x,self.y),self.getEndPoint(),5)
        self.updateHitbox()
        self.updateClicked()
    def drawJoint(self):
        #pygame.draw.rect(screen,(0,0,0),self.hitbox)
        pygame.draw.circle(screen,(255,0,0),(self.x,self.y),jointRadius)
        self.updateHitbox()
        self.updateClicked()
        
    def checkMove(self):
        if self.leftClicked == True:
            self.x = pygame.mouse.get_pos()[0]
            self.y = pygame.mouse.get_pos()[1]
            self.hitbox = pygame.Rect(self.x-hitboxSize/2,self.y-hitboxSize/2,hitboxSize,hitboxSize)
    def checkRotate(self):
        if self.rightClicked:
            xComp = pygame.mouse.get_pos()[0]-self.x
            yComp = pygame.mouse.get_pos()[1]-self.y
            if xComp>0:
                self.globAngle = math.atan(yComp/xComp)
            if xComp<0:
                self.globAngle = math.atan(yComp/xComp)+math.pi
    def rotateAbout(self, target):
        if self.leftClicked: # if clicking, globAngle towards mouse
            xComp = pygame.mouse.get_pos()[0]-target.x
            yComp = pygame.mouse.get_pos()[1]-target.y
            if xComp>0:
                target.globAngle = math.atan(yComp/xComp)
            if xComp<0:
                target.globAngle = math.atan(yComp/xComp)+math.pi
    def updateHitbox(self):
        self.hitbox = pygame.Rect(self.x-hitboxSize/2,self.y-hitboxSize/2,hitboxSize,hitboxSize)
    def updateClicked(self):
        if pygame.mouse.get_pressed()[0] and self.hitbox.collidepoint(pygame.mouse.get_pos()):
            self.leftClicked = True
        if pygame.mouse.get_pressed()[0]==False:
            self.leftClicked = False
        if pygame.mouse.get_pressed()[2] and self.hitbox.collidepoint(pygame.mouse.get_pos()):
            self.rightClicked = True
        if pygame.mouse.get_pressed()[2]==False:
            self.rightClicked = False
    
    

class KinematicChain():
    def __init__(self,chain):
        self.chain = chain
        for pairIdx in range(1,len(self.chain)):
            self.chain[pairIdx].x,self.chain[pairIdx].y = self.chain[pairIdx-1].getEndPoint()
        self.maxLength = 0
        for pairIdx in range(len(self.chain)):
            self.maxLength+=self.chain[pairIdx].length
    def draw(self):
        for pairIdx in range(len(self.chain)-1):
            self.chain[pairIdx].draw()
        self.chain[len(self.chain)-1].drawJoint()
    def checkRotate(self):
        self.chain[0].checkMove()  
        for pairIdx in range(1,len(self.chain)):
            self.chain[pairIdx].x,self.chain[pairIdx].y = self.chain[pairIdx-1].getEndPoint() # arrange pairs into chain
            self.chain[pairIdx].rotateAbout(self.chain[pairIdx-1]) # allow rotation about previous
    def getJacobian(self,movedIdx):
        x = []
        y = []
        for pairIdx in range(movedIdx+1):
            x.append(-1*self.chain[pairIdx].length*math.sin(self.chain[pairIdx].globAngle))
            y.append(self.chain[pairIdx].length*math.cos(self.chain[pairIdx].globAngle))
        return np.array([x,y])
    def getJointAdjustments(self,movedIdx,cinPos):
        cinX,cinY = cinPos
        cinPos = np.array([[cinX],[cinY]])
        return np.dot(np.linalg.pinv(self.getJacobian(movedIdx)),cinPos)
    def calculateIK(self,movedJointIdx,cinPos):
        cinX,cinY = cinPos
        newJoints = self.getJointAdjustments(movedJointIdx,(cinX,cinY))
        self.chain[0].globAngle += newJoints[0]
        self.chain[0].checkMove()
        for i in range(1,movedJointIdx+1):
            self.chain[i].globAngle += newJoints[i]
            self.chain[i].x,self.chain[i].y = self.chain[i-1].getEndPoint()
        for i in range(movedJointIdx+1,len(self.chain)):
            self.chain[i].x,self.chain[i].y = self.chain[i-1].getEndPoint()


def randomChain(size,minLen,maxLen):
    chain = []
    for i in range(size):
        chain.append(KinematicPair((300,200),random.randint(minLen,maxLen),random.random()*2*math.pi))
    return chain


kc = KinematicChain(randomChain(10,50,51))

newMouseX,newMouseY,oldMouseX,oldMouseY = 0,0,0,0

run = True
while run:
    newMouseX,newMouseY = pygame.mouse.get_pos()
    screen.fill((255,255,255))
    for pair in kc.chain:
        if pair.leftClicked:
            kc.calculateIK(kc.chain.index(pair),(newMouseX-oldMouseX,newMouseY-oldMouseY))
    kc.draw()
    pygame.display.flip()
    
    oldMouseX,oldMouseY = pygame.mouse.get_pos()


    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            run = False

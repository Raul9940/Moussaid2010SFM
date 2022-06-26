# -*- coding: utf-8 -*-
"""
Created on Tue Apr 26 19:09:39 2022

@author: Raul
"""


import numpy as np
import matplotlib.pyplot as plt
import math
import sys



def group_distribution(i, lamb):
    j=int(i)
    return np.exp(-lamb)*lamb**j/(math.factorial(j)*(1-np.exp(-lamb)))



AngulosAll=np.zeros(6, dtype=float)
AngulosAll2=np.zeros(6, dtype=float)
DistanciasAll=np.zeros(6, dtype=float)
DistanciasAll2=np.zeros(6, dtype=float)

#nlist contains the values of the number of pedestrians for which the simulations are going to be ran
nlist=np.ones(1, dtype=int)
nlist*=100
allMeanSpeed=[]
allMeanSpeed2=[]
for n in nlist:

    n2=0 #number of groups of size 2
    n3=0 #Number of groups of size 3
    n4=0 #Number of groups of size 4
    vmaxCoef=1.3
    width=5 
    length=14
    tau=0.5
    aparam=10
    b=0.1
    lamb=2
    amayusc=4.5
    gamma=0.35
    nparam=2
    nPrimeparam=3
    phi=np.deg2rad(90)
    beta1=4
    beta2=3
    beta3=1
    PersonalLim=0.8
    hstep=1/20
    tmax=30
    lambProb=0.83
    epsilon=0.005
    a=np.empty((n, 6), dtype=float)
    checkerCondicionesIniciales=True
    counterCheckerCondicionesIniciales=0
    #Initial Conditions of the pedestrians
    while checkerCondicionesIniciales and counterCheckerCondicionesIniciales<101:
        counterCheckerCondicionesIniciales+=1
        if counterCheckerCondicionesIniciales>99:
            print('Unable to fit all pedestrians so they mantain enough distance')


        
        

        ImageCounter=0
        parejas={}
        contadorGlobal=0
        for i in range(n2):

            a[contadorGlobal, 0]=a[contadorGlobal+1, 0]=2*np.random.randint(0,2)-1 
            positionChecker=True
            counterPositionChecker=0
            while positionChecker:
                positionChecker=False
                a[contadorGlobal, 2]=(width-2.0)*np.random.rand()+1.0 #Y position
                a[contadorGlobal ,1]=(length-2.0)*np.random.rand()+1.0 #X position
                for j in range(contadorGlobal):
                    mod=(a[contadorGlobal, 2]-a[j,2])**2+(a[contadorGlobal, 1]-a[j,1])**2
                    if mod<0.35**2 and counterPositionChecker<101:
                        if counterPositionChecker>99:
                            print('Unable to fit all pedestrians so they mantain enough distance')
                        positionChecker=True
                        counterPositionChecker+=1
                    
            positionChecker=True
            counterPositionChecker=0
            while positionChecker:
                positionChecker=False
                theta1=np.random.rand()*(np.pi-0.1)+0.05
                a[contadorGlobal+1, 2]=a[contadorGlobal, 2]+np.sin(theta1)
                a[contadorGlobal+1, 1]=a[contadorGlobal, 1]+np.cos(theta1)
                for j in range(contadorGlobal+1):
                    mod=(a[contadorGlobal+1, 2]-a[j,2])**2+(a[contadorGlobal+1, 1]-a[j,1])**2
                    if mod<0.35**2 and counterPositionChecker<101:
                        if counterPositionChecker>99:
                            print('Unable to fit all pedestrians so they mantain enough distance')
                        positionChecker=True

                        counterPositionChecker+=1
            a[contadorGlobal,3]=a[contadorGlobal, 0]*np.random.normal(1.3, 0.2) #x velocity
            a[contadorGlobal+1, 3]=a[contadorGlobal+1, 0]*np.random.normal(1.3, 0.2)
            a[contadorGlobal,4]=0 #y velocity
            a[contadorGlobal+1, 4]=0
            a[contadorGlobal,5]=abs(a[contadorGlobal, 3]) #Desired velocity
            a[contadorGlobal+1,5]=abs(a[contadorGlobal+1, 3])
            parejas[contadorGlobal]=[contadorGlobal+1]
            parejas[contadorGlobal+1]=[contadorGlobal]
            contadorGlobal+=2
        


     


        for i in range(n3):
            a[contadorGlobal, 0]=a[contadorGlobal+1,0]=a[contadorGlobal+2, 0]=2*np.random.randint(0,2)-1
            positionChecker=True
            counterPositionChecker=0
            while positionChecker:
                positionChecker=False
                a[contadorGlobal, 2]=(width-2.0)*np.random.rand()+1.0 #y position
                a[contadorGlobal ,1]=(length-2.0)*np.random.rand()+1.0 #x position 
                for j in range(contadorGlobal):
                    mod=(a[contadorGlobal, 2]-a[j,2])**2+(a[contadorGlobal, 1]-a[j,1])**2
                    if mod<0.35**2 and counterPositionChecker<101: 
                        if counterPositionChecker>99:
                            print('Unable to fit all pedestrians so they mantain enough distance')
                        
                        positionChecker=True

                        counterPositionChecker+=1
            positionChecker=True
            counterPositionChecker=0
            while positionChecker:
                positionChecker=False
                theta1=np.random.rand()*(np.pi-0.1)+0.05
                a[contadorGlobal+1, 2]=a[contadorGlobal, 2]+np.sin(theta1)
                a[contadorGlobal+1, 1]=a[contadorGlobal, 1]+np.cos(theta1)
                for j in range(contadorGlobal+1):
                    mod=(a[contadorGlobal+1, 1]-a[j,1])**2+(a[contadorGlobal+1, 2]-a[j,2])**2
                    if mod<0.35**2 and counterPositionChecker<101:
                        if counterPositionChecker>99:
                            print('Unable to fit all pedestrians so they mantain enough distance')
                        positionChecker=True
                        counterPositionChecker+=1

            positionChecker=True
            counterPositionChecker=0
            while positionChecker: 
                positionChecker=False
                theta2=np.random.rand()*(np.pi-0.1) + np.pi+0.05
                a[contadorGlobal+2, 2]=a[contadorGlobal, 2]+np.sin(theta2)
                a[contadorGlobal+2, 1]=a[contadorGlobal, 1]+np.sin(theta2)
                for j in range(contadorGlobal+2):
                    mod=(a[contadorGlobal+2, 1]-a[j,1])**2+(a[contadorGlobal+2, 2]-a[j,2])**2
                    if mod<0.35**2 and counterPositionChecker<101:
                        if counterPositionChecker>99:
                            print('Unable to fit all pedestrians so they mantain enough distance')
                        positionChecker=True
                        counterPositionChecker+=1

            a[contadorGlobal,3]=a[contadorGlobal, 0]*np.random.normal(1.3, 0.2) #Velocity in the x axis
            a[contadorGlobal+1, 3]=a[contadorGlobal, 0]*np.random.normal(1.3, 0.2)
            a[contadorGlobal+2, 3]=a[contadorGlobal, 0]*np.random.normal(1.3, 0.2)
            a[contadorGlobal, 4]=0 #Velocity in the Y axis
            a[contadorGlobal+1, 4]=0
            a[contadorGlobal+2, 4]=0
            a[contadorGlobal,5]=abs(a[contadorGlobal, 3]) #Desired velocity
            a[contadorGlobal+1,5]=abs(a[contadorGlobal+1, 3])
            a[contadorGlobal+2, 5]=abs(a[contadorGlobal+2, 3])
            parejas[contadorGlobal]=[contadorGlobal+1, contadorGlobal+2]
            parejas[contadorGlobal+1]=[contadorGlobal, contadorGlobal+2]
            parejas[contadorGlobal+2]=[contadorGlobal, contadorGlobal+1]
            contadorGlobal+=3

        for i in range(n4):
            a[contadorGlobal, 0]=a[contadorGlobal+1,0]=a[contadorGlobal+2, 0]=a[contadorGlobal+3, 0]=2*np.random.randint(0,2)-1
            positionChecker=True
            counterPositionChecker=0
            while positionChecker:
                positionChecker=False
                a[contadorGlobal,2]=(width-2.0)*np.random.rand()+1.0
                a[contadorGlobal, 1]=(length-2.0)*np.random.rand()+1.0
                for j in range(contadorGlobal):
                    mod=(a[contadorGlobal, 1]-a[j,1])**2+(a[contadorGlobal, 2]-a[j,2])**2
                    if mod<0.35**2 and counterPositionChecker<101:
                        if counterPositionChecker>99:
                            print('Unable to fit all pedestrians so they mantain enough distance')
                        positionChecker=True
                        counterPositionChecker+=1

            positionChecker=True
            counterPositionChecker=0
            while positionChecker:
                positionChecker=False
                theta1=np.random.rand()*(np.pi-0.1)+0.05
                a[contadorGlobal+1, 2]=a[contadorGlobal, 2]+np.sin(theta1)
                a[contadorGlobal+1, 1]=a[contadorGlobal, 1]+np.cos(theta1)
                for j in range(contadorGlobal+1):
                    mod=(a[contadorGlobal+1, 1]-a[j,1])**2+(a[contadorGlobal+1, 2]-a[j,2])**2
                    if mod<0.35**2 and counterPositionChecker<101:
                        if counterPositionChecker>99:
                            print('Unable to fit all pedestrians so they mantain enough distance')
                        positionChecker=True

                        counterPositionChecker+=1

            positionChecker=True
            counterPositionChecker=0
            while positionChecker:
                positionChecker=False
                theta2=np.random.rand()*(np.pi-0.1)+0.05+np.pi
                a[contadorGlobal+2, 2]=a[contadorGlobal, 2]+np.sin(theta2)
                a[contadorGlobal+2, 1]=a[contadorGlobal, 1]+np.cos(theta2)
                for j in range(contadorGlobal+2):
                    mod=(a[contadorGlobal+2, 1]-a[j,1])**2+(a[contadorGlobal+2, 2]-a[j,2])**2
                    if mod<0.35**2 and counterPositionChecker<101:
                        if counterPositionChecker>99:
                            print('Unable to fit all pedestrians so they mantain enough distance')
                        positionChecker=True
                        counterPositionChecker+=1        
            positionChecker=True
            counterPositionChecker=0
            while positionChecker:

                positionChecker=False
                theta3=np.random.rand()*(np.pi-0.1)+0.05+np.pi
                a[contadorGlobal+3, 2]=a[contadorGlobal+2, 2]+np.sin(theta3)
                a[contadorGlobal+3, 1]=a[contadorGlobal+2, 1]+np.cos(theta3)
                for j in range(contadorGlobal+3):
                    mod=(a[contadorGlobal+3, 1]-a[j,1])**2+(a[contadorGlobal+3, 2]-a[j,2])**2
                    if mod<0.35**2 and counterPositionChecker<101:
                        if counterPositionChecker>99:
                            print('Unable to fit all pedestrians so they mantain enough distance')
                        positionChecker=True
                        counterPositionChecker+=1        


            
            a[contadorGlobal,3]=a[contadorGlobal, 0]*np.random.normal(1.3, 0.2) #Velocity in the x axis
            a[contadorGlobal+1, 3]=a[contadorGlobal, 0]*np.random.normal(1.3, 0.2)
            a[contadorGlobal+2, 3]=a[contadorGlobal, 0]*np.random.normal(1.3, 0.2)
            a[contadorGlobal+3, 3]=a[contadorGlobal, 0]*np.random.normal(1.3, 0.2)
            a[contadorGlobal, 4]=0 #Velocity in the y axis
            a[contadorGlobal+1, 4]=0
            a[contadorGlobal+2, 4]=0
            a[contadorGlobal+3, 4]=0
            a[contadorGlobal,5]=abs(a[contadorGlobal, 3]) #Desired velocity
            a[contadorGlobal+1,5]=abs(a[contadorGlobal+1, 3])
            a[contadorGlobal+2, 5]=abs(a[contadorGlobal+2, 3])
            a[contadorGlobal+3, 5]=abs(a[contadorGlobal+3, 3])
            parejas[contadorGlobal]=[contadorGlobal+1, contadorGlobal+2, contadorGlobal+3]
            parejas[contadorGlobal+1]=[contadorGlobal, contadorGlobal+2, contadorGlobal+3]
            parejas[contadorGlobal+2]=[contadorGlobal, contadorGlobal+1, contadorGlobal+3]
            parejas[contadorGlobal+3]=[contadorGlobal,contadorGlobal+1, contadorGlobal+2]
            contadorGlobal+=4
        
        restantes=n-contadorGlobal

        for i in range(restantes):
            a[contadorGlobal, 0]=2*np.random.randint(0, 2)-1

            positionChecker=True
            counterPositionChecker=0
            while positionChecker:
                positionChecker=False
                a[contadorGlobal,2]=(width-2.0)*np.random.rand()+1.0 #y position
                a[contadorGlobal, 1]=(length-2.0)*np.random.rand()+1.0 #x position
                for j in range(contadorGlobal):
                    mod=(a[contadorGlobal, 1]-a[j,1])**2+(a[contadorGlobal, 2]-a[j,2])**2
                    if mod<0.35**2 and counterPositionChecker<101:
                        if counterPositionChecker>99:
                            print('Unable to fit all pedestrians so they mantain enough distance')
                        positionChecker=True

                        counterPositionChecker+=1
            a[contadorGlobal,3]=np.random.normal(1.3, 0.2)*a[contadorGlobal, 0] #x velocity
            a[contadorGlobal, 4]=0 # y velocity
            a[contadorGlobal, 5]=abs(a[contadorGlobal, 3]) #Desired velocity
            parejas[contadorGlobal]=[]
            contadorGlobal+=1
        checkerCondicionesIniciales=False
        for i in range(n):
            if a[i,2]<0 or a[i,2]>width:
                checkerCondicionesIniciales=True
            else:
                for j in range(i+1, n):
                    if ((a[i,1]-a[j,1])**2+(a[i,2]-a[j,2])**2)<0.35**2:
                        checkerCondicionesIniciales=True
                        break
                


    t=0
    ResDerecha=[]
    ResIzquierda=[]
    


    angulos=np.zeros(6, dtype=float)
    distancias=np.zeros(6, dtype=float)
    angulos2=np.zeros(6, dtype=float)
    distancias2=np.zeros(6, dtype=float)
    contadorRepeticiones=0
    while t<tmax:   
        xold=a[:,1].copy()
        yold=a[:,2].copy()
        vxold=a[:,3].copy()
        vyold=a[:,4].copy()
        
    
        #FIRST TERM OF THE RUNGE-KUTTA 4 METHOD 
    
        f1x=np.empty(n, dtype=float)
        f1y=np.empty(n, dtype=float)
    
    
        #Goal force
    
        f1x=(a[:,5]*a[:,0]-a[:,3])/tau 
        f1y=(-a[:,4])/tau

        #Pedestrian-Wall repulsion force
        #--Superior wall
        f1y+=-aparam*np.exp(-abs(a[:,2]-width)/b)

        #--Inferior wall
        f1y+=aparam*np.exp(-abs(a[:,2])/b)

    
        #Pedestrian-pedestrian repulsion force
        for i in range(n):
            xaux=a[:,1].copy() 
            for j in range(n):
                if i!=j:
                    if a[j, 1]-a[i,1]<-(length/2):
                        xaux[j]=xaux[j]+length
                    elif a[j,1]-a[i,1]>(length/2):
                        xaux[j]=xaux[j]-length
                    dij=np.sqrt((a[i, 1]-xaux[j])**2+(a[i,2]-a[j,2])**2)                      
                    tijx=lamb*(a[i,3]-a[j,3])+(xaux[j]-a[i,1])/dij                  
                    tijy=lamb*(a[i,4]-a[j,4])+(a[j,2]-a[i,2])/dij     
                    tijmod=np.sqrt(tijx**2+tijy**2)
                    tijx=tijx/tijmod
                    tijy=tijy/tijmod
                    nijx=-tijy
                    nijy=tijx
                    wijx=xaux[j]-a[i,1]
                    wijy=a[j,2]-a[i,2]
                    modwij=np.sqrt(wijx**2+wijy**2)                   
                    thetaij=tijx*wijx/modwij+tijy*wijy/modwij

                    if thetaij>1:
                        print('out of bounds angle, making it 0º')
                        thetaij=1
                    elif thetaij<-1:
                        print('out of bounds angle, making it 180º')
                        thetaij=-1
  
                    thetaij=np.arccos(thetaij) 
                    signothetaij=np.sign(tijx*wijy-tijy*wijx)
                    thetaij*=signothetaij
                    thetaij+=gamma*tijmod*epsilon
                    signothetaij=np.sign(thetaij) 
                    f1x[i]+=-amayusc*np.exp(-dij/(tijmod*gamma))*(np.exp(-(nPrimeparam*gamma*tijmod*thetaij)**2)*tijx+signothetaij*np.exp(-(nparam*gamma*tijmod*thetaij)**2)*nijx)
                    f1y[i]+=-amayusc*np.exp(-dij/(gamma*tijmod))*(np.exp(-(nPrimeparam*gamma*tijmod*thetaij)**2)*tijy+signothetaij*np.exp(-(nparam*gamma*tijmod*thetaij)**2)*nijy)
                    
        #GROUP FORCES
        #--Gazing force
        
            listaParejas=list(parejas.values())[i]
            if len(listaParejas)!=0:
                cix=0
                ciy=0
                for pareja in listaParejas:
                    cix+=xaux[pareja]
                    ciy+=a[pareja, 2]
                cix=cix/len(listaParejas)
                ciy=ciy/len(listaParejas)


                prodEsc=a[i,0]*(cix-a[i,1])

                cimod=np.sqrt((cix-a[i,1])**2+(ciy-a[i,2])**2)
                alfa=np.arccos(prodEsc/(cimod))
                signo=np.sign(a[i,0]*(ciy-a[i,2]))
                alfa*=signo

                if abs(alfa)<=phi: 
                    alfa=0
                else:
                    alfa=signo*(abs(alfa)-phi)
                f1x[i]+=-beta1*abs(alfa)*a[i,3]
                f1y[i]+=-beta1*abs(alfa)*a[i,4]

                #Attractive force
                cyrx=cix-a[i,1]
                cyry=ciy-a[i,2]
                cyrmod=np.sqrt(cyrx**2+cyry**2)
                if cyrmod>len(listaParejas)/2: 
                    f1y[i]+=beta2*cyry/cyrmod
                    f1x[i]+=beta2*cyrx/cyrmod


    
    #Repulsive force
                for element in listaParejas:
                    wikix=xaux[element]-a[i,1]
                    wikiy=a[element, 2]-a[i,2]
                    wikimod=np.sqrt(wikix**2+wikiy**2)
                    if wikimod<PersonalLim:
                        f1x[i]-=beta3*wikix/wikimod
                        f1y[i]-=beta3*wikiy/wikimod

                
        #Part of the code to calculate angles and distances between pedestrians.

            
        if t>10:
            contadorRepeticiones+=1
            nuevoContadorGlobal=0
            xaux=a[:,1].copy()
            contadorParaListas=0
            for k in range(n2):
                listaPosiciones=np.array([[nuevoContadorGlobal, a[nuevoContadorGlobal, 2]],[nuevoContadorGlobal+1, a[nuevoContadorGlobal+1, 2]]])     
                listaPosiciones=listaPosiciones[np.argsort(listaPosiciones[:, 1])]
                p1=int(listaPosiciones[0, 0])
                p2=int(listaPosiciones[1, 0])
                if a[p2, 1]-a[p1,1]<-(length/2):
                    xaux[p2]=xaux[p2]+length
                elif a[p2,1]-a[p1,1]>(length/2):
                    xaux[p2]=xaux[p2]-length
                wijx=xaux[p2]-a[p1, 1]
                wijy=a[p2,2]-a[p1,2]
                distancia=np.sqrt(wijx**2+wijy**2)
                modvelocidad=np.sqrt(a[p1, 3]**2+a[p1, 4]**2)
                angulo=np.arccos((wijx*a[p1, 3]+wijy*a[p1, 4])/(distancia*modvelocidad))
                angulos[contadorParaListas]+=angulo/n2
                distancias[contadorParaListas]+=distancia/n2
                angulos2[contadorParaListas]+=angulo**2/n2
                distancias2[contadorParaListas]+=distancia**2/n2
                nuevoContadorGlobal+=2
            
            for k in range(n3):
                contadorParaListas=1
                listaPosiciones=np.array([[nuevoContadorGlobal, a[nuevoContadorGlobal, 2]],[nuevoContadorGlobal+1, a[nuevoContadorGlobal+1, 2]], [nuevoContadorGlobal+2, a[nuevoContadorGlobal+2, 2]]])
                listaPosiciones=listaPosiciones[np.argsort(listaPosiciones[:, 1])]
                p1=int(listaPosiciones[0,0])
                p2=int(listaPosiciones[1,0])
                p3=int(listaPosiciones[2, 0])
                if a[p2, 1]-a[p1,1]<-(length/2):
                    xaux[p2]=xaux[p2]+length
                elif a[p2,1]-a[p1,1]>(length/2):
                    xaux[p2]=xaux[p2]-length
                wijx=xaux[p2]-a[p1,1]
                wijy=a[p2, 2]-a[p1,2]
                distancia=np.sqrt(wijx**2+wijy**2)
                modvelocidad=np.sqrt(a[p1, 3]**2+a[p1, 4]**2)
                angulo=np.arccos((wijx*a[p1, 3]+wijy*a[p1, 4])/(distancia*modvelocidad))
                angulos[contadorParaListas]+=angulo/n3
                distancias[contadorParaListas]+=distancia/n3
                angulos2[contadorParaListas]+=angulo**2/n3
                distancias2[contadorParaListas]+=distancia**2/n3
                contadorParaListas+=1
                if a[p3, 1]-a[p2,1]<-(length/2):
                    xaux[p3]=xaux[p3]+length
                elif a[p3,1]-a[p2,1]>(length/2):
                    xaux[p3]=xaux[p3]-length
                wijx=xaux[p3]-a[p2,1]
                wijy=a[p3, 2]-a[p2,2]
                distancia=np.sqrt(wijx**2+wijy**2)
                modvelocidad=np.sqrt(a[p2, 3]**2+a[p2, 4]**2)
                angulo=np.arccos((wijx*a[p2, 3]+wijy*a[p2, 4])/(distancia*modvelocidad))
                angulos[contadorParaListas]+=angulo/n3
                distancias[contadorParaListas]+=distancia/n3
                angulos2[contadorParaListas]+=angulo**2/n3
                distancias2[contadorParaListas]+=distancia**2/n3
                contadorParaListas+=1
                nuevoContadorGlobal+=3
            for k in range(n4):
                contadorParaListas=3
                listaPosiciones=np.array([[nuevoContadorGlobal, a[nuevoContadorGlobal, 2]],[nuevoContadorGlobal+1, a[nuevoContadorGlobal+1, 2]], [nuevoContadorGlobal+2, a[nuevoContadorGlobal+2, 2]],[nuevoContadorGlobal+3, a[nuevoContadorGlobal+3, 2]]])
                listaPosiciones=listaPosiciones[np.argsort(listaPosiciones[:, 1])]
                p1=int(listaPosiciones[0,0])
                p2=int(listaPosiciones[1,0])
                p3=int(listaPosiciones[2, 0])
                p4=int(listaPosiciones[3, 0])
                if a[p2, 1]-a[p1,1]<-(length/2):
                    xaux[p2]=xaux[p2]+length
                elif a[p2,1]-a[p1,1]>(length/2):
                    xaux[p2]=xaux[p2]-length
                wijx=xaux[p2]-a[p1,1]
                wijy=a[p2, 2]-a[p1,2]
                distancia=np.sqrt(wijx**2+wijy**2)
                modvelocidad=np.sqrt(a[p1, 3]**2+a[p1, 4]**2)
                angulo=np.arccos((wijx*a[p1, 3]+wijy*a[p1, 4])/(distancia*modvelocidad))
                angulos[contadorParaListas]+=angulo/n4
                distancias[contadorParaListas]+=distancia/n4
                angulos2[contadorParaListas]+=angulo**2/n4
                distancias2[contadorParaListas]+=distancia**2/n4
                contadorParaListas+=1
                if a[p3, 1]-a[p2,1]<-(length/2):
                    xaux[p3]=xaux[p3]+length
                elif a[p3,1]-a[p2,1]>(length/2):
                    xaux[p3]=xaux[p3]-length
                wijx=xaux[p3]-a[p2,1]
                wijy=a[p3, 2]-a[p2,2]
                distancia=np.sqrt(wijx**2+wijy**2)
                modvelocidad=np.sqrt(a[p2, 3]**2+a[p2, 4]**2)
                angulo=np.arccos((wijx*a[p2, 3]+wijy*a[p2, 4])/(distancia*modvelocidad))
                angulos[contadorParaListas]+=angulo/n4
                distancias[contadorParaListas]+=distancia/n4
                angulos2[contadorParaListas]+=angulo**2/n4
                distancias2[contadorParaListas]+=distancia**2/n4
                contadorParaListas+=1
                if a[p4, 1]-a[p3, 1]<-length/2:
                    xaux[p4]=xaux[p4]+length
                elif a[p4, 1]-a[p3, 1]>length/2:
                    xaux[p4]=xaux[p4]-length
                wijx=xaux[p4]-a[p3,1]
                wijy=a[p4, 2]-a[p3,2]
                distancia=np.sqrt(wijx**2+wijy**2)
                modvelocidad=np.sqrt(a[p3, 3]**2+a[p3, 4]**2)
                angulo=np.arccos((wijx*a[p3, 3]+wijy*a[p3, 4])/(distancia*modvelocidad))
                angulos[contadorParaListas]+=angulo/n4
                distancias[contadorParaListas]+=distancia/n4
                angulos2[contadorParaListas]+=angulo**2/n4
                distancias2[contadorParaListas]+=distancia**2/n4
                nuevoContadorGlobal+=4


        g1x=a[:,3].copy()
        g1y=a[:,4].copy()
    

        a[:,1]=xold+g1x*hstep/2
        a[:,2]=yold+g1y*hstep/2
        a[:,3]=vxold+f1x*hstep/2
        a[:,4]=vyold+f1y*hstep/2
        for i in range(n):
            mod=np.sqrt(a[i,3]**2+a[i,4]**2)
            vmax=a[i,5]*vmaxCoef
            if mod>vmax:
                a[i,3]=a[i,3]*vmax/mod
                a[i,4]=a[i,4]*vmax/mod
        
    
        #SECOND TERM OF THE RUNGE-KUTTA 4 METHOD 

        f2x=np.empty(n, dtype=float)
        f2y=np.empty(n, dtype=float)

    
        #Goal force
    
        f2x=(a[:,5]*a[:,0]-a[:,3])/tau 
        f2y=(-a[:,4])/tau


        #Pedestrian-wall repulsive force
        #--Superior wall
        f2y+=-aparam*np.exp(-abs(a[:,2]-width)/b)
        #--Inferior wall
        f2y+=aparam*np.exp(-abs(a[:,2])/b)

        #Pedestrian-pedestrian repulsion force
        for i in range(n):
            xaux=a[:,1]
            for j in range(n):
                if i!=j:
                    if a[j, 1]-a[i,1]<-(length/2):
                        xaux[j]=xaux[j]+length
                    elif a[j,1]-a[i,1]>(length/2):
                        xaux[j]=xaux[j]-length
                    
                    dij=np.sqrt((a[i, 1]-xaux[j])**2+(a[i,2]-a[j,2])**2)
                    tijx=lamb*(a[i,3]-a[j,3])+(xaux[j]-a[i,1])/dij
                    tijy=lamb*(a[i,4]-a[j,4])+(a[j,2]-a[i,2])/dij
                    tijmod=np.sqrt(tijx**2+tijy**2)
                    tijx=tijx/tijmod
                    tijy=tijy/tijmod
                    nijx=-tijy
                    nijy=tijx
                    wijx=xaux[j]-a[i,1]
                    wijy=a[j,2]-a[i,2]
                    wijmod=np.sqrt(wijx**2+wijy**2)
                    thetaij=tijx*wijx/wijmod+tijy*wijy/wijmod
                    
                    if thetaij>1:
                        print('Angle out of bounds. Making it 0º')
                        thetaij=1
                    elif thetaij<-1:
                        print('Angle out of bounds. Making it 180º')
                        thetaij=-1

                    thetaij=np.arccos(thetaij)
                    signothetaij=np.sign(tijx*wijy-tijy*wijx)
                    thetaij*=signothetaij
                    thetaij+=gamma*tijmod*epsilon
                    signothetaij=np.sign(thetaij)
                    f2x[i]+=-amayusc*np.exp(-dij/(tijmod*gamma))*(np.exp(-(nPrimeparam*gamma*tijmod*thetaij)**2)*tijx+signothetaij*np.exp(-(nparam*gamma*tijmod*thetaij)**2)*nijx)
                    f2y[i]+=-amayusc*np.exp(-dij/(gamma*tijmod))*(np.exp(-(nPrimeparam*gamma*tijmod*thetaij)**2)*tijy+signothetaij*np.exp(-(nparam*gamma*tijmod*thetaij)**2)*nijy)

    
    
        #GROUP FORCES
        #--Gazing force

            listaParejas=list(parejas.values())[i]
            if len(listaParejas)!=0:
                cix=0
                ciy=0
                for pareja in listaParejas:
                    cix+=xaux[pareja]
                    ciy+=a[pareja, 2]
                cix=cix/len(listaParejas)
                ciy=ciy/len(listaParejas)
                prodEsc=a[i,0]*(cix-a[i,1])
                cimod=np.sqrt((cix-a[i,1])**2+(ciy-a[i,2])**2)
                alfa=np.arccos(prodEsc/(cimod))
                signo=np.sign(a[i,0]*(ciy-a[i,2]))
                alfa*=signo
                if abs(alfa)<=phi: 
                    alfa=0
                else:
                    alfa=signo*(abs(alfa)-phi)
                f2x[i]+=-beta1*abs(alfa)*a[i,3]
                f2y[i]+=-beta1*abs(alfa)*a[i,4]
                #Attractive force
                cyrx=cix-a[i,1]
                cyry=ciy-a[i,2]
                cyrmod=np.sqrt(cyrx**2+cyry**2)
                if cyrmod>len(listaParejas)/2:
                    f2x[i]+=beta2*cyrx/cyrmod
                    f2y[i]+=beta2*cyry/cyrmod
                    
    #Repulsive force
                for element in listaParejas:
                    wikix=xaux[element]-a[i,1]
                    wikiy=a[element, 2]-a[i,2]
                    wikimod=np.sqrt(wikix**2+wikiy**2)
                    if wikimod<PersonalLim:
                        f2x[i]-=beta3*wikix/wikimod
                        f2y[i]-=beta3*wikiy/wikimod
    
        g2x=a[:,3].copy()
        g2y=a[:,4].copy()
    
        a[:,1]=xold+g2x*hstep/2
        a[:,2]=yold+g2y*hstep/2
        a[:,3]=vxold+f2x*hstep/2
        a[:,4]=vyold+f2y*hstep/2
        for i in range(n):
            mod=np.sqrt(a[i,3]**2+a[i,4]**2)
            vmax=a[i,5]*vmaxCoef
            if mod>vmax:
                a[i,3]=a[i,3]*vmax/mod
                a[i,4]=a[i,4]*vmax/mod
    
    
        #THIRD TERM OF THE RUNGE-KUTTA 4 METHOD 
    
        f3x=np.empty(n, dtype=float)
        f3y=np.empty(n, dtype=float)
    
    
        #Goal force
    
        f3x=(a[:,5]*a[:,0]-a[:,3])/tau
        f3y=(-a[:,4])/tau


        #Pedestrian-wall repulsive force
        #--Superior wall
        f3y+=-aparam*np.exp(-abs(a[:,2]-width)/b)
        #--Inferior wall
        f3y+=aparam*np.exp(-abs(a[:,2])/b)


    
        #Pedstrian-pedestrian repulsive force
        for i in range(n):
            xaux=a[:,1]
            for j in range(n):
                if i!=j:
                    if a[j, 1]-a[i,1]<-(length/2):
                        xaux[j]=xaux[j]+length
                    elif a[j,1]-a[i,1]>length/2:
                        xaux[j]=xaux[j]-length
                    dij=np.sqrt((a[i, 1]-xaux[j])**2+(a[i,2]-a[j,2])**2)
                    tijx=lamb*(a[i,3]-a[j,3])+(xaux[j]-a[i,1])/dij
                    tijy=lamb*(a[i,4]-a[j,4])+(a[j,2]-a[i,2])/dij
                    tijmod=np.sqrt(tijx**2+tijy**2)
                    tijx=tijx/tijmod
                    tijy=tijy/tijmod
                    nijx=-tijy
                    nijy=tijx
                    wijx=xaux[j]-a[i,1]
                    wijy=a[j,2]-a[i,2]
                    wijmod=np.sqrt(wijx**2+wijy**2)
                    thetaij=tijx*wijx/wijmod+tijy*wijy/wijmod
                    
                    if thetaij>1:
                        print('Angle out of bounds. Making it 0º')
                        thetaij=1
                    elif thetaij<-1:
                        print('Angle out of bounds. Making it 180º')
                        thetaij=-1

                    thetaij=np.arccos(thetaij)
                    signothetaij=np.sign(tijx*wijy-tijy*wijx)
                    thetaij*=signothetaij
                    thetaij+=gamma*tijmod*epsilon
                    signothetaij=np.sign(thetaij)

                    f3x[i]+=-amayusc*np.exp(-dij/(tijmod*gamma))*(np.exp(-(nPrimeparam*gamma*tijmod*thetaij)**2)*tijx+signothetaij*np.exp(-(nparam*gamma*tijmod*thetaij)**2)*nijx)
                    f3y[i]+=-amayusc*np.exp(-dij/(gamma*tijmod))*(np.exp(-(nPrimeparam*gamma*tijmod*thetaij)**2)*tijy+signothetaij*np.exp(-(nparam*gamma*tijmod*thetaij)**2)*nijy)

                    
    
        #GROUP FORCES
        #--Gazing force

            listaParejas=list(parejas.values())[i]
            if len(listaParejas)!=0: #Esto significa que hay parejas
                cix=0
                ciy=0
                for pareja in listaParejas:
                    cix+=xaux[pareja]
                    ciy+=a[pareja, 2]
                cix=cix/len(listaParejas)
                ciy=ciy/len(listaParejas)
                prodEsc=a[i,0]*(cix-a[i,1])

                cimod=np.sqrt((cix-a[i,1])**2+(ciy-a[i,2])**2)
                alfa=np.arccos(prodEsc/(cimod))
                signo=np.sign(a[i,0]*(ciy-a[i,2]))
                alfa*=signo

                if abs(alfa)<=phi: #Si esta fuera del rango entonces su valor se mantiene
                    alfa=0
                else:
                    alfa=signo*(abs(alfa)-phi)
    
                f3x[i]+=-beta1*abs(alfa)*a[i,3]
                f3y[i]+=-beta1*abs(alfa)*a[i,4]
                #Attractive force
                cyrx=cix-a[i,1]
                cyry=ciy-a[i,2]
                cyrmod=np.sqrt(cyrx**2+cyry**2)

                if cyrmod>len(listaParejas)/2:
                    f3x[i]+=beta2*cyrx/cyrmod
                    f3y[i]+=beta2*cyry/cyrmod
    
        #Repulsive Force
                for element in listaParejas:
                    wikix=xaux[element]-a[i,1]
                    wikiy=a[element, 2]-a[i,2]
                    wikimod=np.sqrt(wikix**2+wikiy**2)
                    if wikimod<PersonalLim:
                        f3x[i]-=beta3*wikix/wikimod
                        f3y[i]-=beta3*wikiy/wikimod
    
        g3x=a[:,3].copy()
        g3y=a[:,4].copy()
    
    
    
    
    
        a[:,1]=xold+g3x*hstep
        a[:,2]=yold+g3y*hstep
        a[:,3]=vxold+f3x*hstep
        a[:,4]=vyold+f3y*hstep
        for i in range(n):
            mod=np.sqrt(a[i,3]**2+a[i,4]**2)
            vmax=a[i,5]*vmaxCoef
            if mod>vmax:
                a[i,3]=a[i,3]*vmax/mod
                a[i,4]=a[i,4]*vmax/mod
    
    
    
        #FOURTH TERM OF THE RUNGE-KUTTA 4 METHOD 
    
        f4x=np.empty(n, dtype=float)
        f4y=np.empty(n, dtype=float)
    
    
        #Goal force
    
        f4x=(a[:,5]*a[:,0]-a[:,3])/tau
        f4y=(-a[:,4])/tau
    
        #Pedestrian-wall repulsive force
        #--Superior wall
        f4y+=-aparam*np.exp(-abs(a[:,2]-width)/b)
        #--Inferior wall
        f4y+=aparam*np.exp(-abs(a[:,2])/b)
    
        #Pedestrian-pedestrian repulsive force
        for i in range(n):
            for j in range(n):
                if i!=j:
                    if a[j, 1]-a[i,1]<-(length/2):
                        xaux[j]=xaux[j]+length
                    elif a[j,1]-a[i,1]>(length/2):
                        xaux[j]=xaux[j]-length
                    dij=np.sqrt((a[i, 1]-xaux[j])**2+(a[i,2]-a[j,2])**2)
                    tijx=lamb*(a[i,3]-a[j,3])+(xaux[j]-a[i,1])/dij
                    tijy=lamb*(a[i,4]-a[j,4])+(a[j,2]-a[i,2])/dij
                    tijmod=np.sqrt(tijx**2+tijy**2)
                    tijx=tijx/tijmod
                    tijy=tijy/tijmod
                    nijx=-tijy
                    nijy=tijx
                    wijx=xaux[j]-a[i,1]
                    wijy=a[j,2]-a[i,2]
                    wijmod=np.sqrt(wijx**2+wijy**2)
                    thetaij=tijx*wijx/wijmod+tijy*wijy/wijmod
                    
                    if thetaij>1:
                        print('Angle out of bounds. Making it 0º')
                        thetaij=1
                    elif thetaij<-1:
                        print('Angle out of bounds. Making it 180º')
                        thetaij=-1
                    thetaij=np.arccos(thetaij)
                    signothetaij=np.sign(tijx*wijy-tijy*wijx)
                    thetaij*=signothetaij
                    thetaij+=gamma*tijmod*epsilon
                    signothetaij=np.sign(thetaij)
                    f4x[i]+=-amayusc*np.exp(-dij/(tijmod*gamma))*(np.exp(-(nPrimeparam*gamma*tijmod*thetaij)**2)*tijx+signothetaij*np.exp(-(nparam*gamma*tijmod*thetaij)**2)*nijx)
                    f4y[i]+=-amayusc*np.exp(-dij/(gamma*tijmod))*(np.exp(-(nPrimeparam*gamma*tijmod*thetaij)**2)*tijy+signothetaij*np.exp(-(nparam*gamma*tijmod*thetaij)**2)*nijy)
    
    
    
        #GROUP FORCES
        #--Gazing force
            listaParejas=list(parejas.values())[i]
            if len(listaParejas)!=0:
                cix=0
                ciy=0
                for pareja in listaParejas:
                    cix+=xaux[pareja]
                    ciy+=a[pareja, 2]
                cix=cix/len(listaParejas)
                ciy=ciy/len(listaParejas)
                prodEsc=a[i,0]*(cix-a[i,1])

                cimod=np.sqrt((cix-a[i,1])**2+(ciy-a[i,2])**2)
                alfa=np.arccos(prodEsc/(cimod))
                signo=np.sign(a[i,0]*(ciy-a[i,2]))
                alfa*=signo
                if abs(alfa)<=phi: 
                    alfa=0
                else:
                    alfa=signo*(abs(alfa)-phi)
    
                f4x[i]+=-beta1*abs(alfa)*a[i,3]
                f4y[i]+=-beta1*abs(alfa)*a[i,4]

                #Attractive force
                cyrx=cix-a[i,1]
                cyry=ciy-a[i,2]
                cyrmod=np.sqrt(cyrx**2+cyry**2)
                if cyrmod>len(listaParejas)/2:
                    f4x[i]+=beta2*cyrx/cyrmod
                    f4y[i]+=beta2*cyry/cyrmod

    
                #Repulsive force
                for element in listaParejas:
                    wikix=xaux[element]-a[i,1]
                    wikiy=a[element, 2]-a[i,2]
                    wikimod=np.sqrt(wikix**2+wikiy**2)
                    if wikimod<PersonalLim:
                        f4x[i]-=beta3*wikix/wikimod
                        f4y[i]-=beta3*wikiy/wikimod

    
        g4x=a[:,3].copy()
        g4y=a[:,4].copy()
        for i in range(n):
            if np.isnan(f1x[i]) or np.isnan(f1y[i]) or np.isnan(f2x[i]) or np.isnan(f2y[i]) or np.isnan(f3y[i]) or np.isnan(f3x[i]) or np.isnan(f4x[i]) or np.isnan(f4y[i]):
                print('Some force goes to infinity')
    
        a[:,1]=xold+hstep/6*(g1x + 2*g2x + 2*g3x+g4x)
        a[:,2]=yold+hstep/6*(g1y + 2*g2y+ 2*g3y+g4y)
        a[:,3]=vxold+hstep/6*(f1x + 2*f2x + 2*f3x+f4x)
        a[:,4]=vyold+hstep/6*(f1y+ 2*f2y + 2*f3y+f4y)
        for i in range(n):
            mod=np.sqrt(a[i,3]**2+a[i,4]**2)
            vmax=a[i,5]*vmaxCoef
            if mod>vmax:
                a[i,3]=a[i,3]*vmax/mod
                a[i,4]=a[i,4]*vmax/mod





        
        for i in range(n):
            if a[i,1]>length:
                a[i,1]-=length
            elif a[i,1]<0:
                a[i,1]+=length

            if a[i,2]>=width:
                a[i,2]=width-0.05
            elif a[i,2]<=0:
                a[i,2]=width+0.05
            if np.isnan(a[i,1]):
                a[i,1]=xold[i]+hstep*vxold[i] 
            if np.isnan(a[i,2]):
                a[i,2]=yold[i]+hstep*vyold[i]
            if np.isnan(a[i,3]):
                a[i,3]=vxold[i]
            if np.isnan(a[i,4]):
                a[i,4]=vyold[i]

        t+=hstep
        if t>10:
           SumSpeeds=np.sqrt(a[:,3]**2+a[:,4]**2)
           meanSpeed=sum(SumSpeeds)/len(SumSpeeds)
           allMeanSpeed.append(meanSpeed)
           allMeanSpeed2.append(meanSpeed**2)

        if t>10:
            auxCounter=-1
            while auxCounter<n-1:
                
                auxCounter+=1
                if len(parejas[auxCounter])>-1:
    
                    lista=[]
                    mod=np.sqrt(a[auxCounter,3]**2+a[auxCounter,4]**2)
                    lista.append([a[auxCounter,1], a[auxCounter,2], mod])
        
                    try:
        
                        for i in range(len(parejas[auxCounter])):
                            auxCounter+=1
                            mod=np.sqrt(a[auxCounter,3]**2+a[auxCounter,4]**2)
                            lista.append([a[auxCounter,1], a[auxCounter,2], mod])
                        nplista=np.array(lista, dtype=float)
                        if a[auxCounter, 0]==-1:
                            plt.plot(nplista[:,0], nplista[:,1], markersize=5*nplista[0,2], color='red', marker='o')
                        else:
                            plt.plot(nplista[:,0], nplista[:,1], markersize=5*nplista[0,2], color='blue', marker='o')
                    except KeyError:
                        if a[auxCounter, 0]==-1:
                            plt.plot(nplista[0,0], nplista[0,1], markersize=10*nplista[0,2],color='red',marker='o')
                        else:
                            plt.plot(nplista[0,0], nplista[0,1], markersize=10*nplista[0,2],color='blue',marker='o')
    
    
            ImageCounter+=1
            plt.title('t={}'.format(t))
            plt.savefig('imagenGruposNuevas{}.png'.format(ImageCounter))

            plt.show()
    

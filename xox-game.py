import random
botc = [1,2,3,4,5,6,7,8,9]

choice = random.choice(botc)

row1 = ["-","-","-"]

row2 = ["-","-","-"]

row3 = ["-","-","-"]



bot = "x"
def rows():
    print(" ".join(row1))
    print(" ".join(row2))
    print(" ".join(row3))
    print("***********")


user2 = " "

used = []

def hata():
    print("Tanımsız veri girdiniz!")
    
def match():
    
    for i in range(1,10):
        
        global used
        global user2
        
        user2 = int(user2)
        botc.remove(user2)
        matchResult()
        
def giris():

    
    print("Koordinat seçiniz giriniz (0->bitir)")
    print("1->(0,0) ,2->(0,1) , 3->(0,2)")
    print("4->(1,0) ,5->(1,1) , 6->(1,2)")
    print("7->(2,0) ,8->(2,1) , 9->(2,2)")
    global user2
    user2 = input("")
    try:
        user2 = int(user2)
    except:
       return user2
    
    
def matchResult():
    
    global user2
    
    user2 = int(user2)
    
    if user2<4 and user2>0:
        
        row1[(user2-1)]=user1
        
    if user2<7 and user2>3:
        
        row2[(user2-4)]=user1
        
    if user2<10 and user2>6:
        
        row3[(user2-7)]=user1

def matchResult2():
    
    if choice<4 and choice>0:
        
        row1[(choice-1)]=bot
        
    if choice<7 and choice>3:
        
        row2[(choice-4)]=bot
        
    if choice<10 and choice>6:
        
        row3[(choice-7)]=bot
        
        
        
def dene():
    
    try:
        
        giris()
    
    except:
        
        hata()
        giris()
        
#




while True:
    user1 = input("x o?")
    if user1=="x":
        
        bot="o"
    user1 = user1.lower()
    if user1=="x" or user1=="o":
        
        while True:
                      
            giris()
               
            if user2==0:
                break
            elif user2 in used:
                hata()
            elif user2==1 or user2==2 or user2==3 or user2==4 or user2==5 or user2==6 or user2==7 or user2==8 or user2==9:
                used.append(user2)
                matchResult()
                rows()
                botc.remove(user2)
                try:
                    choice = random.choice(botc)
                except:
                    continue
                print(f"Bilgisayar {choice} seçti")
                used.append(choice)
                botc.remove(choice)
                matchResult2()
                rows()
            else:
                hata()
                    
                    
            if row1[0]==row2[0] and row2[0]==row3[0]:#sütun1
                    
                if row1[0]=="x":
                        
                    if user1=="x":
                        rows()
                        print("Tebrikler Kazandınız")
                        break
                    else:
                        rows()
                        print("maalesef kaybettiniz")
                        break
                        
                if row1[0]=="o":
                            
                       if user1=="o":
                            rows()
                            print("Tebrikler Kazandınız")
                            break
                       else:
                            rows()
                            print("maalesef kaybettiniz")
                            break
                        
                    
            if row1[1]==row2[1] and row2[1]==row3[1]:#sütun2
                
                if row1[1]=="x":
                    
                    if user1=="x":
                        rows()
                        print("Tebrikler Kazandınız")
                        break
                    else:
                        rows()
                        print("maalesef kaybettiniz")
                        break
                    
                if row1[1]=="o":
                        
                        if user1=="o":
                            rows()
                            print("Tebrikler Kazandınız")
                            break
                        else:
                            rows()
                            print("maalesef kaybettiniz")
                            break
                    
            if row1[2]==row2[2] and row2[2]==row3[2]:#sütun3
               
                if row1[2]=="x":
                    
                    if user1=="x":
                        rows()
                        print("Tebrikler Kazandınız")
                        break
                    else:
                        rows()
                        print("maalesef kaybettiniz")
                        break
                    
                if row1[2]=="o":
                   
                    if user1=="o":
                       
                        rows()
                        print("Tebrikler Kazandınız")
                        break
                    else:
                        rows()
                        print("maalesef kaybettiniz")
                        break
                        
            if row1[0]==row1[1] and row1[1]==row1[2]:#satır1
                
                 if row1[0]=="x":
                     
                    if user1=="x":
                        rows()
                        print("Tebrikler Kazandınız")
                        break
                    else:
                        rows()
                        print("maalesef kaybettiniz")
                        break
                        
                 if row1[0]=="o":
                          
                       if user1=="o":
                            rows()
                            print("Tebrikler Kazandınız")
                            break
                       else:
                            rows()
                            print("maalesef kaybettiniz")
                            break
                        
            if row2[0]==row2[2] and row2[0]==row2[1]:#satır2
                   
                if row2[0]=="x":
                       
                    if user1=="x":
                        rows()
                        print("Tebrikler Kazandınız")
                        break
                    else:
                        rows()
                        print("maalesef kaybettiniz")
                        break
                   
                if row2[0]=="o":
                        
                        if user1=="o":
                            rows()
                            print("Tebrikler Kazandınız")
                            break
                        else:
                            rows()
                            print("maalesef kaybettiniz")
                            break
                        
            if row3[0]==row3[1] and row3[0]==row3[2]:#satır3
                    
                if row3[0]=="x":
                        
                    if user1=="x":
                        rows()
                        print("Tebrikler Kazandınız")
                        break
                    else:
                        rows()
                        print("maalesef kaybettiniz")
                        break
                        
                if row3[0]=="o":
                         
                        if user1=="o":
                            rows()
                            print("Tebrikler Kazandınız")
                            break
                        else:
                            rows()
                            print("maalesef kaybettiniz")
                            break
                        
            if row1[0]==row2[1] and row2[1]==row3[2]:#diagonal
                    
                if row1[0]=="x":
                        
                    if user1=="x":
                        rows()
                        print("Tebrikler Kazandınız")
                        break
                    else:
                        rows()
                        print("maalesef kaybettiniz")
                        break
                        
                if row1[0]=="o":
                            
                        if user1=="o":
                            rows()
                            print("Tebrikler Kazandınız")
                            break
                        else:
                            rows()
                            print("maalesef kaybettiniz")
                            break
                        
                
            if row1[2]==row2[1] and row2[1]==row3[0]:#antidiagonal
                    
                if row1[2]=="x":
                        
                    if user1=="x":
                        rows()
                        print("Tebrikler Kazandınız")
                        break
                    else:
                        rows()
                        print("maalesef kaybettiniz")
                        break
                        
                if row1[2]=="o":
                            
                        if user1=="o":
                            rows()
                            print("Tebrikler Kazandınız")
                            break
                        else:
                            rows()
                            print("maalesef kaybettiniz")
                            break
            print(used)
    elif user1=="0":
        break
    else:
        hata()


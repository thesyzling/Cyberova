import random
row1 = ["-", "-", "-"]
row2 = ["-", "-", "-"]
row3 = ["-", "-", "-"]
user1 = input("select x or o?")
user2 = ""
com = ""
choice =""
winning = ""
used = []
def harf_secimi():
    global com
    if user1 == "x":
        com = "o"
        return com
    elif user1 == "o":
        com = "x"
        return com
    return com
def kazanma_şartları():
    global winning
    if row1[0] == row1[1] and row1[0] == row1[2]:   #satır1
        if row1[0]=="o" and row2[0]=="o":
            winning ="o"
            return winning
        elif row1[0]=="x" and row1[2]=="x":
            winning = "x"
            return winning
    elif row2[0] == row2[1] and row2[0] == row2[2]:   #satır2
        if row2[0]=="o"and row2[2]=="o":
            winning ="o"
            return winning
        elif row2[0]=="x"and row2[2]=="x":
            winning = "x"
            return winning
    elif row3[0] == row3[1] and row3[0] == row3[2]:   #satır3
        if row3[0]=="o" and row3[2]=="o":
            winning ="o"
            return winning
        elif row3[0]=="x"and row3[2]=="x":
            winning = "x"
            return winning
    elif row1[0] == row2[1] and row2[1] == row3[2]:    #diagonal
        if row1[0]=="o" and row2[1]=="o":
            winning="o"
            return winning
        elif row1[0] =="x" and row3[2]=="x":
            winning = "x"
            return winning
    elif row1[2] == row2[1] and row2[1] == row3[0]:     #ters diagonal
        if row1[2]=="o" and row2[1]=="o":
            winning="o"
            return winning
        elif row1[2]=="x"and row3[0]=="x":
            winning = "x"
            return winning
    elif row1[0]==row2[0] and row1[0]==row3[0]:      #sütün1
        if row1[0]=="o" and row3[0]=="o":
            winning="o"
            return winning
        elif row1[0]=="x" and row3[0]=="x":
            winning = "x"
            return winning
    elif row1[1]==row2[1] and row1[1]==row3[1]:       #sütün2
        if row1[1]=="o" and row2[1]=="o":
            winning="o"
            return winning
        elif row1[1]=="x" and row3[1]=="x":
            winning = "x"
            return winning
    elif row1[2]==row2[2] and row1[2]==row3[2]:      #sütün3
        if row1[2]=="o" and row2[2]=="o":
            winning="o"
            return winning
        elif row1[2]=="x" and row3[2]=="x":
            winning = "x"
            return winning
def kullanicinin_harfini_yerleştirme():   # burada kendi seçimimi yerleştiriyorum
    if user2 < 4 and user2 > 0:
        row1[user2 - 1] = user1
        return user2 - 1
    elif user2 < 7 and user2 > 3:
        row2[user2 - 4] = user1
        return user2 - 4
    elif user2 < 10 and user2 > 6:
        row3[user2 - 7] = user1
        return user2-7
    else:
        print("lütfen sadece 0-9 arasında değer giriniz")
def bilgisayarin_harfini_yerlestirme():

    if choice < 4 and choice > 0:      # burada bilgisayarın seçimini yerleştiriyorum.
       row1[choice - 1] = com
       return choice -1
    elif choice < 7 and choice > 3:
       row2[choice - 4] = com
       return choice-4
    elif choice < 10 and choice > 6:
       row3[choice - 7] = com
       return choice-7
    else:
       print("1ve 9 arası değer gir")
    return
harf_secimi()
if user1=="x":
    while True:
        user2 =int(input("Koordinat seçiniz giriniz (0->bitir)\n1->(0,0) ,2->(0,1) , 3->(0,2)\n4->(1,0) ,5->(1,1) , 6->(1,2)\n7->(2,0) ,8->(2,1) , 9->(2,2)"))
        if user2 ==0:
            break
        elif user2 in used:
            print("tanımsız veri girdiniz.")
            continue
        choice = random.randint(1,9)
        used.append(choice)
        used.append(user2)
        print(user2)
        print(choice)
        kullanicinin_harfini_yerleştirme()
        bilgisayarin_harfini_yerlestirme()
        kazanma_şartları()
        print(" ".join(row1))
        print(" ".join(row2))
        print(" ".join(row3))
        if winning=="x":
            print("kazandınız!")
            break
        elif winning =="o":
            print("oyun bitti, maalesef kaybettiniz!")
            break

elif user1=="o":
    while True:
        choice = random.randint(1, 9)
        bilgisayarin_harfini_yerlestirme()
        print(" ".join(row1))
        print(" ".join(row2))
        print(" ".join(row3))
        user2=int(input("Koordinat seçiniz giriniz (0->bitir)\n1->(0,0) ,2->(0,1) , 3->(0,2)\n4->(1,0) ,5->(1,1) , 6->(1,2)\n7->(2,0) ,8->(2,1) , 9->(2,2)"))
        print(user2)
        kullanicinin_harfini_yerleştirme()
        print(" ".join(row1))
        print(" ".join(row2))
        print(" ".join(row3))
        print("--------------")
        kazanma_şartları()
        if winning =="x":
            print("oyun bitti, maalesef kaybettiniz!")
            break
        elif winning =="o":
            print("kazandınız!")
            break



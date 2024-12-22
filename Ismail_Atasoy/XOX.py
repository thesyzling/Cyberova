import time

counter = 0

row1 = ["*","*","*"]
row2 = ["*","*","*"]
row3 = ["*","*","*"]

p1 = "x"
p2 = "o"

player1 = input("Enter your nickname player1 !!! ")
player2 = input("Enter your nickname player2 !!! ")

cordinates = [0,1,2,3,4,5,6,7,8,9]

used = []

def user1_input():
    print("Enter the cordinate.   (To exit enter 0)")
    print(cordinates)
    print("1->(*) 2->(*) 3->(*)")
    print("4->(*) 5->(*) 6->(*)")
    print("7->(*) 8->(*) 9->(*)")


    global user1
    user1 = int(input(f"Make your move {player1} ! "))

def user2_input():
    print("Enter the cordinate.   (To exit enter 0)")
    print(cordinates)
    print("1->(*) 2->(*) 3->(*)")
    print("4->(*) 5->(*) 6->(*)")
    print("7->(*) 8->(*) 9->(*)")


    global user2
    user2 = int(input(f"Now, your turn {player2} ! "))

def try_input1():
    try:
         user1_input()
         if user1 in used:
             error()
             try_input1()
         elif user1 not in cordinates:
             error()
             try_input1()
         else:
             used.append(user1)
    except:
        error()
        try_input1()

def try_input2():
    try:
        user2_input()
        if user2 in used:
            error()
            try_input1()
        elif user2 not in cordinates:
            error()
            try_input2()
        else:
            used.append(user2)
    except:
        error()
        try_input2()

def fill_the_blank1():
    global user1
    user1 = int(user1)

    if user1 > 0 and user1 < 4:
        row1[(user1 - 1)] = p1
    if user1 > 3 and user1 < 7:
        row2[(user1 - 4)] = p1
    if user1 > 6 and user1<10:
        row3[(user1 - 7)] = p1

def fill_the_blank2():
    global user2
    user2 = int(user2)

    if user2 > 0 and user2 < 4:
        row1[(user2 - 1)] = p2
    elif user2 > 3 and user2 < 7:
        row2[(user2 - 4)] = p2
    elif user2 > 6 and user2 < 10:
        row3[(user2 - 7)] = p2

def error():
    print("You have entered undefined data !!! ")

def board_v():
    print("".join(row1))
    print("".join(row2))
    print("".join(row3))
    print("-------------------------")

while True:
    try_input1()
    if user1 == 0:
        break
    cordinates.remove(user1)
    fill_the_blank1()
    board_v()
    if row1 == ["x", "x", "x"]:
        print(f"{player1} WON !!!")
        break
    if row2 == ["x", "x", "x"]:
            print(f"{player1} WON !!!")
            break
    if row3 == ["x", "x", "x"]:
        print(f"{player1} WON !!!")
        break
    if row1[0] == "x" and row1[0] == row2[0] and row2[0] == row3[0]:
            print(f"{player1} WON !!!")
            break
    if row1[1] == "x" and row1[1] == row2[1] and row2[1] == row3[1]:
        print(f"{player1} WON !!!")
        break
    if row1[2] == "x" and row1[2] == row2[2] and row2[2] == row3[2]:
        print(f"{player1} WON !!!")
        break
    if row1[0] == "x" and row1[0] == row2[1] and row2[1] == row3[2]:
        print(f"{player1} WON !!!")
        break
    if row1[2] == "x" and row1[2] == row2[1] and row2[1] == row3[0]:
        print(f"{player1} WON !!!")
        break
    if cordinates == [0]:
        print("The game has no winner !")
        break

    try_input2()
    if user2 == 0:
        break
    cordinates.remove(user2)
    fill_the_blank2()
    board_v()
    if row1 == ["o", "o", "o"]:
        print(f"{player2} WON !!!")
        break
    if row2 == ["o", "o", "o"]:  # r2
        print(f"{player2} WON !!!")
        break
    if row3 == ["o", "o", "o"]:
        print(f"{player2} WON !!!")
        break
    if row1[0] == "o" and row1[0] == row2[0] and row2[0] == row3[0]:
        print(f"{player2} WON !!!")
        break
    if row1[1] == "o" and row1[1] == row2[1] and row2[1] == row3[1]:
        print(f"{player2} WON !!!")
        break
    if row1[2] == "o" and row1[2] == row2[2] and row2[2] == row3[2]:
        print(f"{player2} WON !!!")
        break
    if row1[0] == "o" and row1[0] == row2[1] and row2[1] == row3[2]:
        print(f"{player2} WON !!!")
        break
    if row1[2] == "o" and row1[2] == row2[1] and row2[1] == row3[0]:
        print(f"{player2} WON !!!")
        break
    if row1 == ["o","o","o"]:
        print(f"{player2} WON !!!")
        break
    if row2 == ["o","o","o"]:   #r2
        print(f"{player2} WON !!!")
        break
    if row3 == ["o", "o", "o"]:
        print(f"{player2} WON !!!")
        break
    if row1[0] == "o" and row1[0] == row2[0] and row2[0] == row3[0]:
        print(f"{player2} WON !!!")
        break
    if row1[1] == "o" and row1[1] == row2[1] and row2[1] == row3[1]:
        print(f"{player2} WON !!!")
        break
    if row1[2] == "o" and row1[2] == row2[2] and row2[2] == row3[2]:
        print(f"{player2} WON !!!")
        break
    if row1[0] == "o" and row1[0] == row2[1] and row2[1] == row3[2]:
        print(f"{player2} WON !!!")
        break
    if row1[2] == "o" and row1[2] == row2[1] and row2[1] == row3[0]:
        print(f"{player2} WON !!!")
        break

time.sleep(20)





































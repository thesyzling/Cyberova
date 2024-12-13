gameBoard = [" " for x in range(10)]

def showScreen():
    print(" " + gameBoard[1] + " " + "|" + " " + gameBoard[2] + " " + "|" + " " + gameBoard[3])
    print("----------")
    print(" " + gameBoard[4] + " " + "|" + " " + gameBoard[5] + " " + "|" + " " + gameBoard[6])
    print("----------")
    print(" " + gameBoard[7] + " " + "|" + " " + gameBoard[8] + " " + "|" + " " + gameBoard[9])

def put(letter, loc):
    gameBoard[loc] = letter

def look(loc):
    return gameBoard[loc] == " "

def gameBoardlook():
    if gameBoard.count(" ") > 1:
        return False
    else:
        return True

def winner(letter):
    return (gameBoard[1] == gameBoard[2] == gameBoard[3] == letter) or (gameBoard[4] == gameBoard[5] == gameBoard[6] == letter) or (gameBoard[7] == gameBoard[8] == gameBoard[9] == letter) or (gameBoard[1] == gameBoard[4] == gameBoard[7] == letter) or (gameBoard[2] == gameBoard[5] == gameBoard[8] == letter) or (gameBoard[3] == gameBoard[6] == gameBoard[9] == letter) or (gameBoard[1] == gameBoard[5] == gameBoard[9] == letter) or (gameBoard[3] == gameBoard[5] == gameBoard[7] == letter)

def playerMove():
    loc = int(input("1-9 arasında bir değer giriniz: "))
    if look(loc):
        put("X", loc)
        if winner("X"):
            showScreen()
            print("Tebrikler kazandınız!!!")
            exit()
        showScreen()
    else:
        print("Girdiğiniz alan dolu, Tekrar seçiniz.")
        playerMove()

import random

def computerMove():
    available_loc = [loc for loc, letter in enumerate(gameBoard) if letter == " " and loc != 0]
    if available_loc:
        loc = random.choice(available_loc)
        put("O", loc)
        if winner("O"):
            showScreen()
            print("Bilgisayar kazandı!!!")
            exit()
        showScreen()


print("Tic Tac Toe Oyununa Hoşgeldiniz.")
showScreen()

while not gameBoardlook():
    playerMove()
    if gameBoardlook():
        print("Oyun bitti kazanan yok.")
        exit()
    computerMove()
    if gameBoardlook():
        print("Oyun bitti kazanan yok.")
        exit()



#XOX Oyunu 

oyunTahtasi = [" " for x in range(10)]

def ekranaGoster():
    print(" " +oyunTahtasi[1] + " "+ "|" + " " +oyunTahtasi[2] + " " +"|" + " " + oyunTahtasi[3])
    print("----------")
    print(" " +oyunTahtasi[4] + " "+ "|" + " " +oyunTahtasi[5] + " " +"|" + " " + oyunTahtasi[6])
    print("----------")
    print(" " +oyunTahtasi[7] + " "+ "|" + " " +oyunTahtasi[8] + " " +"|" + " " + oyunTahtasi[9])
  
    

def harfkoy (harf,konum):
    oyunTahtasi[konum] = harf

def alan_bos_mu (konum):
    return oyunTahtasi[konum] == " "

def tahta_dolu():
    if oyunTahtasi.count(" ") > 1 :
        return False
    return True

def kazanan(oyunTahtasi,harf):
    return ( oyunTahtasi[1] == harf and oyunTahtasi[2] == harf and oyunTahtasi[3] == harf) or (oyunTahtasi[4] == harf and oyunTahtasi[5] == harf and oyunTahtasi[6] == harf) or (oyunTahtasi[7] == harf and oyunTahtasi[8] == harf and oyunTahtasi[9] == harf )or ( oyunTahtasi[1] == harf and oyunTahtasi[5] == harf and oyunTahtasi[9] == harf) or ( oyunTahtasi[3] == harf and oyunTahtasi[5] == harf and oyunTahtasi[7] == harf) or ( oyunTahtasi[3] == harf and oyunTahtasi[6] == harf and oyunTahtasi[9] == harf) or ( oyunTahtasi[1] == harf and oyunTahtasi[4] == harf and oyunTahtasi[7] == harf) or ( oyunTahtasi[2] == harf and oyunTahtasi[5] == harf and oyunTahtasi[8] == harf) 



def oyuncu_hareketi():
    konum= int(input(" 1 ile 9 arasinda bir konum giriniz: "))
    if alan_bos_mu(konum):
        harfkoy("X",konum)
        if kazanan(oyunTahtasi,"X"):
            ekranaGoster()
            print("Tebrikler! kazandiniz.")
            exit()
        ekranaGoster()
    else:
        print("Girdiginiz konum dolu.Tekrar konum seciniz: ")
        oyuncu_hareketi()
        

def bilgisayar_hareket():
    import random
    musait_konumlar = [konum for konum, harf in enumerate(oyunTahtasi) if harf == " " and konum != 0] 

    hareket= 0


    for harf in ["X","O"]:
        for i in musait_konumlar:
            kopya_tahta = oyunTahtasi[:]
            kopya_tahta[i] = harf
            if kazanan(kopya_tahta,harf):
                hareket= i 
                return hareket

    koseler=[]
    for i in musait_konumlar:
        if i in [1,3,7,9]:
            koseler.append(i)

    if len(koseler) > 0:
        hareket = random.choice(koseler)
        return hareket

    if 5 in musait_konumlar:
        hareket = 5
        return hareket


    iceri_kisim= []

    for i in musait_konumlar:
        if i in [2,4,6,8]:
            iceri_kisim.append(i)

    if len(iceri_kisim) > 0:
        hareket = random.choice(iceri_kisim)
        return hareket


def oyun():
    print(" XOX Oyununa hosgeldiniz!:)")
    print("Her bir satir soldan saga dogru 1 den baslayarak 9 a kadar numarlandirilmisitir.")
    ekranaGoster()


    while not tahta_dolu():
        oyuncu_hareketi()
        if tahta_dolu():
            print("oyun bitti, kazanan yok.")
            exit()
            

        print("----------------------")

        bilgisayarin_hareketi = bilgisayar_hareket()
        harfkoy("O",bilgisayarin_hareketi)
        if kazanan(oyunTahtasi,"O"):
            ekranaGoster()
            print("Bilgisayar kazandi,tekrar deneyiniz.")
            exit()

        ekranaGoster()
        if tahta_dolu():
            print("oyun bitti, kazanan yok.")
            exit()
            

        print("----------------------")
oyun()





  


                   
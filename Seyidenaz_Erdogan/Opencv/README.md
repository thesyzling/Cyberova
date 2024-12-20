OPENCV NEDİR?
OpenCV (Open Source Computer Vision Library, anlamı Açık Kaynak Bilgisayar Görüsü Kütüphanesi) gerçek-zamanlı bilgisayar görüsü uygulamalarında kullanılan açık kaynaklı kütüphanedir. İlk olarak Intel tarafından geliştirilmiş, daha sonra Willow Garage ve sonra Itseez (Intel tarafından satın alındı) tarafından sürdürülmüştür.
OpenCv kütüphanesi ile görüntüler üzerinde; dönüşümler, tonlamalar, renk odakları, arka plan ayrıştırma, yüz tanıma, nesne tanıma, yaya tanıma, araç sayma, araç tanıma gibi uygulamalar yaparak görüntülerin nasıl analiz edilebileceğini öğrenebiliriz.

OpenCV Kullanım Alanları
•	Yüzleri algılama ve tanıma
•	Nesneleri tanımlama
•	Videolarda insan eylemlerini sınıflandırma
•	Kamera hareketlerini takip etme
•	Hareket eden nesneleri takip etme
•	Nesnelerin 3B modellerini çıkarma
•	Bir resim veritabanından benzer resimler bulma
•	Göz hareketlerini takip etme
•	Manzarayı tanıma
•	Mimik tanıma
       OPENCV BİLEŞENLERİ
CORE= Opencv’nin temel fonksiyonları ve matris, point, size gibi veri yapılarını bulundurur.
HIGHGUI= Resim görüntüleme, pencereleri yönetme ve grafiksel kullanıcı arabilimleri için gerekli olan metotları barındırır.
IMGPROC= Filtreleme operatörleri, kenar bulma, nesne belirleme, renk uzayı yönetimi, renk yönetimi ve eşikleme gibi neredeyse tüm fonksiyonları içine alan bir pakettir.
IMGCODECS=Dosya sistemi üzerinden resim ve video oluşturma/ yazma işlemlerini yerine getiren metotları barındırmaktadır.
VIDEOIO= Kameralara ve video cihazlarına erişmek, görüntü almak ve görüntü yazmak için gerekli metotları barındırır.
OPENCV RESİM OKUMA VE YAZMA İŞLEMİ
Resme ait pixel ve renk değerlerinin okunarak hafızada saklanması anlamına gelir.
imread  Dosyayı okur.
imshowResmi açar.
waitKey  Resmin ekranda ne kadar süre kalacağını ayarlamamızı sağlar.0 yazdığımızda herhangi bir işlem yapmadığımız sürece resim ekranda kalır.
destroyAllWindows Açık olan bütün pencereleri kapatır.
IMREAD_GREYSCALE  Resmi siyah-beyaz yapar.
İmwrite Resim dosyasını oluşturur.

KAMERA AÇMA
cv2.videoCapture  Bu, bilgisayarınıza bağlı olan ilk kamerayı (genellikle yerleşik kamera) kullanarak bir video akışı başlatır.Eğer başka bir kamera kullanmak isterseniz, 0 yerine 1, 2 gibi diğer indeksleri deneyebilirsiniz.
vid.read  bir video kaynağından (örneğin, bir kamera veya video dosyası) bir kareyi okur ve iki değer döndürür:
ret (return) bir bool değerdir ve kameradan başarılı bir şekilde veri alınıp alınmadığını belirtir:
•	True: Eğer kamera bir kareyi başarıyla okuduysa.
•	False: Eğer kamera bir hata nedeniyle kareyi okuyamazsa (örneğin, kamera bağlantısı kesilmişse).
frame, kameradan alınan görüntü verisinin kendisidir. Görüntü üzerindeki tüm işlemler (örneğin, görüntüleme, filtre uygulama veya kaydetme) bu değişken üzerinde yapılır.
      Cv2.flip  görüntüyü yatay olarak ters çevirir.
        1: Yatay çevirme (ayna efekti).
        0: Dikey çevirme.
       -1: Hem yatay hem dikey çevirme.
cv2.imshow()  Alınan ve çevrilen görüntüyü bir pencere içinde gösterir.
cv2.waitKey  Her kare arasında beklemek için kullanılır.
vid.release  Kamera kaynağını serbest bırakır.
Cv2.destroyAllWindows Tüm OpenCV pencerelerini kapatır.















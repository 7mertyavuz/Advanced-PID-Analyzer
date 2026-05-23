# 🎛️ PID Kontrol ve Optimizasyon Simülatörü

Bu proje, dinamik sistemlerin modellenmesi, PID kontrolör parametrelerinin otomatik hesaplanması (auto-tuning) ve sistem kararlılık analizlerinin yapılması için geliştirilmiş bir Python masaüstü uygulamasıdır. Kontrol sistemleri teorisini yazılımla pratiğe dökmek amacıyla tasarlanmıştır.

## 🚀 Projenin Amacı
Sistemlerin transfer fonksiyonlarını analiz ederken PID katsayılarını deneme-yanılma yoluyla bulmak yerine; sayısal optimizasyon algoritmaları kullanarak en uygun Kp, Ki ve Kd değerlerini otonom bir şekilde hesaplayan bir araç geliştirmektir.

## ✨ Temel Özellikler
*   **⚡ Otomatik PID Optimizasyonu:** `SciPy` (Nelder-Mead algoritması) kullanılarak Hata Kareleri İntegralini (ISE) minimize eden en ideal parametreleri bulur. Yüksek aşımı (overshoot) engellemek için ceza fonksiyonları içerir.
*   **📊 Mühendislik Analiz Grafikleri:**
    *   **Basamak Yanıtı (Step Response):** Sistemin hedefe ulaşma süresini ve aşım miktarını gösterir.
    *   **Bode Diyagramı:** Sistemin frekans yanıtını analiz eder.
    *   **Kutup-Sıfır Haritası (Pole-Zero Map):** Sistemin karmaşık düzlemdeki kararlılığını doğrular.
    *   **Hata Grafiği (Tracking Error):** Hedef değer ile anlık sistem durumu arasındaki sapmayı zamanla gösterir.
*   **🌙 Gelişmiş Arayüz:** Tkinter ve Matplotlib entegrasyonu ile geliştirilmiş, anlık simülasyon sonuçlarını gösteren karanlık tema (dark mode) tasarımı.

## 🛠️ Kurulum ve Çalıştırma

1. **Projeyi bilgisayarınıza indirin:**
```bash
   git clone [https://github.com/KULLANICI_ADIN/Advanced-PID-Suite.git](https://github.com/KULLANICI_ADIN/Advanced-PID-Suite.git)
   cd Advanced-PID-Suite
Gerekli kütüphaneleri kurun:

Bash
   pip install numpy scipy matplotlib control
Uygulamayı çalıştırın:

Bash
   python pid_suite.py
💻 Nasıl Kullanılır?
Sol panele sisteminizin Transfer Fonksiyonu (Pay ve Payda) katsayılarını girin. (Örn: 1 / (s^2 + 3s + 2) için Pay: 1, Payda: 1, 3, 2).

Başlangıç için PID Katsayılarını (Kp, Ki, Kd) ve Hedef (Setpoint) değerini belirleyin.

Manuel analiz için "▶ SIMÜLASYONU ÇALIŞTIR" butonuna tıklayın ve grafikleri inceleyin.

Algoritmanın en uygun katsayıları kendi kendine hesaplaması için "⚡ PID OPTİMİZE ET" butonuna tıklayın.

🧰 Kullanılan Teknolojiler
python-control: Kontrol teorisi matematiği ve transfer fonksiyonu işlemleri.

scipy.optimize: Maliyet fonksiyonu minimizasyonu ve optimizasyon.

matplotlib: Arayüz içine gömülü dinamik grafik çizimleri.

numpy & tkinter: Matris hesaplamaları ve arayüz tasarımı.

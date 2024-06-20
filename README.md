### Disusun dan di rancang oleh
Kelompok 6 TMK Mobile Robot:
M Fat Hiy Ilman N 			(210491100003)
Astri Nur P 				(210491100004)
Siti Nurhalisa 			(210491100017)
Devan Yusfa S 			(210491100027)
Firman Aliyansyah S 		(210491100030)


![Screenshot 2024-06-20 190757](https://github.com/devanys/PRM-mobile-robot/assets/145944367/3fc0b7c1-e906-4cfd-8062-7faf953a64fa)


![image](https://github.com/devanys/PRM-mobile-robot/assets/145944367/90ea5fc5-ac98-4d7b-aa30-3aa4d42a4b1e)


### Implementasi Probabilistic Roadmap (PRM) untuk Mobile Robot

Probabilistic Roadmap (PRM) adalah metode yang digunakan dalam perencanaan lintasan untuk robot mobile di lingkungan yang kompleks dan dinamis. Pendekatan ini menggabungkan konsep pemetaan lingkungan dengan teknik pengambilan sampel acak untuk membangun representasi grafis dari ruang konfigurasi robot. Tujuan utama dari PRM adalah untuk menciptakan jaringan titik-titik (nodes) yang terhubung oleh lintasan (edges) yang aman dan memungkinkan robot untuk bergerak dari posisi awal ke tujuan dengan efisien. melibatkan langkah-langkah berikut:

1. **Pemetaan Lingkungan**:
   - **Pemrosesan Data**: Data dari sensor diolah untuk membangun representasi digital lingkungan sekitar robot.

2. **Pembentukan Probabilistic Roadmap**:
   - **Node**: PRM menggunakan titik-titik sampel yang ditempatkan secara acak di ruang konfigurasi robot, mewakili posisi mulai, tujuan, dan titik-titik antara.
   - **Hubungan Jalan**: Untuk setiap titik sampel, algoritma menentukan koneksi jalan antara titik-titik yang dapat dilewati tanpa bertabrakan dengan penghalang.

3. **Pencarian Lintasan Optimal**:
   - **Algoritma Pencarian**: Algoritma seperti A* atau Dijkstra digunakan untuk mencari lintasan optimal antara titik awal dan tujuan dalam graf yang terdiri dari titik-titik sampel sebagai simpul dan koneksi jalan sebagai tepi.
![image](https://github.com/devanys/PRM-mobile-robot/assets/145944367/cbcb24ea-b4d2-42c4-acb3-3b54052e2b04)
![image](https://github.com/devanys/PRM-mobile-robot/assets/145944367/b28f5f82-0ae5-4134-8002-ceaaa912ac6d)

4. **Implementasi pada Robot Mobile**:
   - **Kontrol Robot**: Lintasan yang dihasilkan dieksekusi oleh sistem kontrol robot untuk menggerakkan robot dari titik awal ke tujuan, mempertimbangkan dinamika robot dan hambatan lain.
   - **Pembaruan Lintasan**: Robot dapat memperbarui perencanaan lintasan jika ada perubahan dalam lingkungan yang terdeteksi.

Implementasi PRM memungkinkan robot mobile untuk melakukan perencanaan lintasan dengan efisien dalam lingkungan yang kompleks dan berubah-ubah. Dengan memanfaatkan sampel acak dan algoritma pencarian, PRM dapat menangani tantangan navigasi dengan mempertimbangkan ketepatan, kecepatan, dan keamanan pergerakan robot.

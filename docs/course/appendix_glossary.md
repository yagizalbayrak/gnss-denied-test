<div align="center">

# 📚 Ek · Sözlük, Formül Kartı ve Kaynaklar

[← Ders 6](06_evaluation_and_showcase.md) · [İçindekiler](README.md)

</div>

---

## 🧮 Formül kartı

Bütün kursun matematiği tek sayfada. Sınavdan önce bakılan kâğıt gibi kullan.

### Geodesy ve frame'ler (Ders 1)

| Ne? | Formül |
|---|---|
| Prime vertical radius | $N(\varphi) = a / \sqrt{1 - e^2\sin^2\varphi}$ |
| Geodetic → ECEF | $X = (N+h)\cos\varphi\cos\lambda,\ \ Y = (N+h)\cos\varphi\sin\lambda,\ \ Z = (N(1-e^2)+h)\sin\varphi$ |
| ECEF → ENU | $\mathbf{p}_{\text{ENU}} = \mathbf{R}(\varphi_0,\lambda_0)\,(\mathbf{p} - \mathbf{p}_0)$ |
| Düz Dünya hatası | $\Delta h \approx d^2 / 2R_\oplus$ |
| Rotation özellikleri | $\mathbf{R}^\top\mathbf{R} = \mathbf{I}$, $\det\mathbf{R} = 1$, $\mathbf{R}^{-1} = \mathbf{R}^\top$ |
| SE(3) tersi | $T^{-1} = [\mathbf{R}^\top,\ -\mathbf{R}^\top\mathbf{t}]$ |
| Frame zinciri | $T_{a\,c} = T_{a\,b}\,T_{b\,c}$ |
| Covariance yayılımı | $\boldsymbol{\Sigma}_y = \mathbf{J}\,\boldsymbol{\Sigma}_x\,\mathbf{J}^\top$ |

### Gürültü modelleri (Ders 2)

| Ne? | Formül |
|---|---|
| White noise, sürekli → ayrık | $\sigma_d = \sigma_c / \sqrt{\Delta t}$ |
| Random walk adımı | $\sigma_{\text{step}} = \sigma_c\,\sqrt{\Delta t}$ |
| Rayleigh medyanı (CEP) | $1.177\,\sigma$ |
| Cauchy kuyruğu | $P(\lvert x\rvert > k\gamma) = 1 - \tfrac{2}{\pi}\arctan k$ |
| Spoofing ramp | $\mathbf{z} = \mathbf{p} + v_s(t - t_0)\,(\sin\beta, \cos\beta, 0)$ |

### IMU (Ders 3)

| Ne? | Formül |
|---|---|
| Specific force | $\mathbf{f}_b = \mathbf{R}^\top(\mathbf{a}_w - \mathbf{g}_w)$ |
| Rodrigues (Exp) | $\text{Exp}(\boldsymbol{\phi}) = \mathbf{I} + \tfrac{\sin\theta}{\theta}[\boldsymbol{\phi}]_\times + \tfrac{1-\cos\theta}{\theta^2}[\boldsymbol{\phi}]_\times^2$ |
| Log açısı | $\theta = \arccos\big((\text{tr}\,\mathbf{R} - 1)/2\big)$ |
| Strapdown rotation | $\mathbf{R}_{k+1} = \mathbf{R}_k\,\text{Exp}((\tilde{\boldsymbol{\omega}} - \mathbf{b}_g)\Delta t)$ |
| Strapdown velocity | $\mathbf{v}_{k+1} = \mathbf{v}_k + \mathbf{g}\Delta t + \mathbf{R}_k(\tilde{\mathbf{a}} - \mathbf{b}_a)\Delta t$ |
| Accel bias hatası | $\delta p = \tfrac{1}{2}b_a t^2$ |
| Gyro bias → tilt hatası | $\delta p = \tfrac{1}{6}g\,b_g t^3$ |
| Roll / pitch (sabit) | $\phi = \operatorname{atan2}(f_y, f_z)$, $\theta = \operatorname{atan2}(-f_x, \sqrt{f_y^2+f_z^2})$ |

### Factor graph (Ders 4)

| Ne? | Formül |
|---|---|
| MAP → least squares | $\mathcal{X}^\ast = \arg\min \sum_i \lVert h_i(\mathcal{X}_i) - \mathbf{z}_i\rVert^2_{\boldsymbol{\Sigma}_i}$ |
| Mahalanobis | $\lVert\mathbf{e}\rVert^2_{\boldsymbol{\Sigma}} = \mathbf{e}^\top\boldsymbol{\Sigma}^{-1}\mathbf{e}$ |
| Gauss-Newton | $(\mathbf{J}^\top\mathbf{J})\,\boldsymbol{\delta} = -\mathbf{J}^\top\mathbf{e}$ |
| Retraction | $\mathbf{R}\oplus\boldsymbol{\delta} = \mathbf{R}\,\text{Exp}(\boldsymbol{\delta})$ |
| Preintegrated rotation | $\Delta\tilde{\mathbf{R}}_{ij} = \prod_k \text{Exp}((\tilde{\boldsymbol{\omega}}_k - \mathbf{b}_g)\Delta t) \approx \mathbf{R}_i^\top\mathbf{R}_j$ |
| Preintegrated velocity | $\Delta\tilde{\mathbf{v}}_{ij} \approx \mathbf{R}_i^\top(\mathbf{v}_j - \mathbf{v}_i - \mathbf{g}\Delta t_{ij})$ |
| Bias random walk factor | $\mathbf{b}_j - \mathbf{b}_i \sim \mathcal{N}(\mathbf{0}, \sigma_{rw}^2\Delta t_{ij})$ |
| Condition number | $\kappa(\mathbf{J}^\top\mathbf{J}) = \kappa(\mathbf{J})^2$ |

### Görüntü (Ders 5)

| Ne? | Formül |
|---|---|
| Pinhole | $u = f_x X/Z + c_x$, $v = f_y Y/Z + c_y$ |
| Stereo derinlik | $Z = f\,b / d$ |
| Derinlik hatası | $\sigma_Z = Z^2\sigma_d / (f\,b)$ |
| PnP | $\arg\min_{\mathbf{T}} \sum_i \lVert \mathbf{u}_i - \pi(\mathbf{K}, \mathbf{T}\mathbf{P}_i)\rVert^2$ |
| RANSAC iterasyonu | $N = \log(1-p) / \log(1-w^s)$ |
| Essential matrix | $\mathbf{x}_2^\top[\mathbf{t}]_\times\mathbf{R}\,\mathbf{x}_1 = 0$ (ölçek belirsiz) |
| Kamera → body hareketi | $\mathbf{T}_{b_{k-1}b_k} = \mathbf{T}_{bc}\,\mathbf{T}_{c_{k-1}c_k}\,\mathbf{T}_{cb}$ |
| Huber ağırlığı | $w(r) = 1$ ($\lvert r\rvert\le k$), $k/\lvert r\rvert$ (aksi), $k = 1.345$ |
| Attention | $\text{softmax}(\mathbf{Q}\mathbf{K}^\top / \sqrt{d})\,\mathbf{V}$ |

### Değerlendirme (Ders 6)

| Ne? | Formül |
|---|---|
| ATE | $\sqrt{\tfrac{1}{N}\sum\lVert\hat{\mathbf{p}}_i - \mathbf{p}_i\rVert^2}$ |
| RPE | $\mathbf{E}_i = (\mathbf{T}_i^{-1}\mathbf{T}_{i+\Delta})^{-1}(\hat{\mathbf{T}}_i^{-1}\hat{\mathbf{T}}_{i+\Delta})$ |
| NEES | $\varepsilon = \mathbf{e}^\top\boldsymbol{\Sigma}^{-1}\mathbf{e} \sim \chi^2_3$, $\mathbb{E}[\varepsilon] = 3$ |
| 3σ elipsoid | $\boldsymbol{\Sigma} = \mathbf{V}\boldsymbol{\Lambda}\mathbf{V}^\top$, yarı eksenler $3\sqrt{\lambda_i}$ |
| Innovation gate | $\boldsymbol{\nu}^\top\mathbf{S}^{-1}\boldsymbol{\nu} > \chi^2_{3,0.99} = 11.34$ |

---

## 📖 Sözlük

Terimler İngilizce, açıklamalar Türkçe. Parantez içindeki sayı, terimin ilk anlatıldığı ders.

### A–F

| Terim | Açıklama |
|---|---|
| **Absolute sensor** (0) | Konumu doğrudan ölçen sensör (GNSS). Hatası birikmez. |
| **Alignment** (6) | Değerlendirmeden önce tahmini gerçeğe en iyi oturtan dönüşümü uygulamak. |
| **Allan variance** (2) | Uzun sabit kayıttan IMU gürültü parametrelerini çıkarma yöntemi. |
| **ARW**, Angle Random Walk (2) | Gyro white noise'unun açıdaki etkisi, °/√h. |
| **ATE**, Absolute Trajectory Error (6) | Tahmin ile gerçek konum arasındaki farkın RMSE'si. |
| **Attention** (5) | Bir öğenin diğerlerine ne kadar "bakacağını" öğrenen mekanizma; transformer'ların temeli. |
| **Baseline** (5) | Stereo kameralar arası mesafe. |
| **Bayes tree** (4) | ISAM2'nin kullandığı, faktorize edilmiş graph'ın ağaç yapısı. |
| **Bias** (2) | IMU ölçümündeki yavaş değişen sistematik kayma. |
| **Bias random walk** (2) | Bias'ın zamanla rastgele gezinmesi. |
| **Body frame** (0) | Araca sabit koordinat sistemi; burada x ileri, y sol, z yukarı. |
| **CEP**, Circular Error Probable (2) | Yatay hataların yarısının içinde kaldığı yarıçap. |
| **Cholesky / QR** (4) | Lineer sistem çözme yöntemleri; QR sayısal olarak daha kararlı. |
| **Common random numbers** (2) | Senaryoları karşılaştırırken aynı rastgele sayıları kullanarak varyansı azaltma. |
| **Condition number** (4) | Bir matrisin sayısal hassasiyeti; büyükse küçük hatalar büyür. |
| **Conjugation** (5) | $\mathbf{T}\mathbf{M}\mathbf{T}^{-1}$: bir hareketi başka bir frame'de ifade etmek. |
| **Consistency** (6) | Estimator'ın raporladığı belirsizliğin gerçek hatayla uyumu. |
| **Covariance** (0) | Çok boyutlu belirsizliğin büyüklüğü ve şekli. |
| **Dead reckoning** (0) | Sadece göreli ölçümlerle konumu ileri taşımak. |
| **Descriptor** (5) | Bir keypoint'in eşleştirme için kullanılan parmak izi. |
| **Disparity** (5) | Aynı noktanın sol ve sağ görüntüdeki yatay piksel farkı. |
| **Drift** (0) | Relative sensörlerde hatanın zamanla birikmesi. |
| **ECEF** (1) | Earth-Centered Earth-Fixed; Dünya merkezli kartezyen sistem. |
| **EKF** (4) | Extended Kalman Filter; geçmişi marginalize eden tek adımlık estimator. |
| **ENU** (1) | East-North-Up yerel teğet düzlem frame'i. |
| **Essential matrix** (5) | İki kamera görüntüsü arasındaki epipolar ilişki; ölçeği bilinmez. |
| **Euler angles** (1) | Roll, pitch, yaw ile rotasyon temsili. |
| **Exp / Log map** (3) | so(3) ↔ SO(3) arasındaki dönüşümler. |
| **Factor** (4) | Bir ölçümün değişkenler üzerine koyduğu olasılık / ceza terimi. |
| **Factor graph** (4) | Değişkenler ve factor'lerden oluşan iki parçalı graph. |
| **First-order hold** (3) | İki komşu IMU örneğinin ortalamasıyla entegrasyon. |

### G–P

| Terim | Açıklama |
|---|---|
| **Gauss-Newton** (4) | Nonlinear least squares için doğrusallaştır-çöz-güncelle iterasyonu. |
| **Geodetic latitude** (1) | Ellipsoid normalinin ekvator düzlemiyle yaptığı açı. |
| **Gimbal lock** (1) | Euler açılarında bir serbestlik derecesinin kaybolduğu tekillik. |
| **GNSS** (0) | Global Navigation Satellite System: GPS, Galileo, GLONASS, BeiDou. |
| **Huber loss** (5) | Küçük hatalarda karesel, büyük hatalarda doğrusal büyüyen robust maliyet. |
| **IMU** (0) | Inertial Measurement Unit: accelerometer + gyroscope. |
| **Information matrix** (4) | Covariance'ın tersi; $\mathbf{J}^\top\mathbf{J}$. |
| **Initial alignment** (3) | Harekete başlamadan başlangıç yönelimini bulma. |
| **Inlier / outlier** (5) | Modelle uyumlu / uyumsuz ölçüm. |
| **Innovation** (4) | Gelen ölçüm ile tahminin farkı. |
| **Intrinsics** (5) | Kameranın iç parametreleri: $f_x, f_y, c_x, c_y$. |
| **IRLS** (5) | Iteratively Reweighted Least Squares; robust kernel'ların çözüm yöntemi. |
| **ISAM2** (4) | Incremental Smoothing and Mapping; factor graph'ı artımlı çözen algoritma. |
| **Jacobian** (1) | Çok değişkenli fonksiyonun türev matrisi. |
| **Jamming** (0) | GNSS frekansında gürültü basarak sinyali bastırma. |
| **Keyframe** (4) | Graph'ta değişken tutulan zaman anı. |
| **Keypoint** (5) | Görüntüde belirgin, tekrar bulunabilir nokta. |
| **Levenberg-Marquardt** (4) | Sönümlemeli Gauss-Newton. |
| **Lie algebra / Lie group** (3) | Düz tangent space (so(3)) ve onun "sarıldığı" eğri grup (SO(3)). |
| **LIS**, Longest Increasing Subsequence (2) | Dizinin en uzun artan alt dizisi. |
| **MAGSAC** (5) | Gürültü ölçeği üzerinden marginalize eden RANSAC varyantı. |
| **Mahalanobis distance** (4) | Covariance ile normalize edilmiş, "kaç sigma" cinsinden mesafe. |
| **Manifold** (3) | Yerel olarak düz görünen eğri uzay (SO(3), SE(3)). |
| **MAP** (4) | Maximum A Posteriori; verilen ölçümlerle en olası durum. |
| **Marginal covariance** (4) | Ortak dağılımdan bir değişkenin kendi belirsizliği. |
| **Monte Carlo** (6) | Aynı deneyi farklı rastgele sayılarla çok kez çalıştırmak. |
| **Multipath** (0) | GNSS sinyalinin yansıyarak gecikmeli gelmesi. |
| **NEES** (6) | Normalized Estimation Error Squared; consistency testi. |
| **Noise density** (2) | Sürekli zaman gürültü şiddeti, birim/√Hz. |
| **Observability** (4) | Bir değişkenin ölçümlerden belirlenebilir olması. |
| **ORB** (5) | Oriented FAST + Rotated BRIEF; hızlı, ikili feature. |
| **Pinhole model** (5) | Kameranın ideal izdüşüm modeli. |
| **PnP** (5) | Perspective-n-Point; 3D-2D eşleşmelerden kamera pozu. |
| **Preintegration** (4) | Keyframe'ler arası IMU ölçümlerini tek bir göreli ölçüme sıkıştırma. |
| **Prior** (4) | Ölçümlerden önceki bilgi; aynı zamanda regularization. |
| **Proprioceptive / exteroceptive** (0) | Kendi içini / dış dünyayı ölçen sensör. |
| **Pseudorange** (0) | Saat hatası içeren uydu-alıcı mesafesi. |

### Q–Z

| Terim | Açıklama |
|---|---|
| **Quaternion** (1) | Rotasyonun 4 sayılık temsili; projede wxyz sırası. |
| **RANSAC** (5) | Rastgele minimal örneklerle outlier'a dayanıklı model uydurma. |
| **Reprojection error** (5) | 3D noktanın görüntüye düşürülmesi ile ölçülen piksel arasındaki fark. |
| **Rectification** (5) | Stereo görüntüleri eşleşmeler aynı satırda olacak şekilde yeniden örnekleme. |
| **Relative sensor** (0) | Sadece hareketi ölçen sensör (IMU, VO). Hatası birikir. |
| **Residual** (4) | Tahmin edilen ile ölçülen arasındaki fark. |
| **Retraction** (3) | Tangent space'teki bir adımı manifold'a geri taşıma. |
| **RMSE** (6) | Root Mean Square Error. |
| **Robust kernel** (5) | Outlier'ların etkisini sınırlayan maliyet fonksiyonu. |
| **RPE**, Relative Pose Error (6) | Kısa pencere içindeki hareketin hatası. |
| **RTK** (2) | Real-Time Kinematic; santimetre doğruluklu GNSS düzeltmesi. |
| **Schuler oscillation** (3) | INS hatalarının ≈ 84 dakikalık periyotla salınması. |
| **SE(3) / SO(3)** (1) | Rigid body transform / rotation grupları. |
| **Skew-symmetric matrix** (3) | $[\boldsymbol{\omega}]_\times$; cross product'ı matris olarak yazar. |
| **Smoothing** (4) | Geçmiş durumları sonradan gelen ölçümlerle de düzeltmek. |
| **Sparsity** (4) | Matrisin çoğunun sıfır olması; hızlı çözümün anahtarı. |
| **Specific force** (3) | Accelerometer'ın ölçtüğü, yerçekimi dışındaki kuvvetlerden gelen ivme. |
| **Spoofing** (0) | Sahte GNSS sinyaliyle alıcıyı yanlış konuma inandırma. |
| **Strapdown** (3) | IMU'nun araca sabit olduğu, dönüşlerin matematikle takip edildiği INS. |
| **Turn-on bias** (2) | IMU her açıldığında farklı olan başlangıç bias'ı. |
| **Visual odometry** (5) | Kamera görüntülerinden hareket tahmini. |
| **Whitening** (4) | Residual'ı covariance'ın karekökünün tersiyle çarparak birimsiz yapmak. |
| **WGS84** (1) | GNSS'in referans ellipsoid'i. |

---

## 📚 Kaynaklar: bir sonraki adım

Kurstaki her konunun "asıl kaynağı". Önerilen okuma sırasıyla.

| Konu | Kaynak | Neden oku? |
|---|---|---|
| 🌍 Navigasyonun tamamı | P. Groves, *Principles of GNSS, Inertial, and Multisensor Integrated Navigation Systems* | Alanın "incil"i. GNSS, INS, hata modelleri. |
| 📐 Lie grupları | J. Solà ve ark., *A micro Lie theory for state estimation in robotics* (2018) | Exp/Log, retraction, Jacobian'lar; 30 sayfada. |
| 🕸️ Factor graph | F. Dellaert, M. Kaess, *Factor Graphs for Robot Perception* (2017) | GTSAM'in yazarlarından. |
| 🕸️ ISAM2 | M. Kaess ve ark., *iSAM2: Incremental Smoothing and Mapping Using the Bayes Tree* (IJRR 2012) | Projedeki solver. |
| ⚙️ Preintegration | C. Forster ve ark., *On-Manifold Preintegration for Real-Time Visual-Inertial Odometry* (IEEE T-RO 2017) | `ImuFactor`'ün teorisi. |
| 📊 State estimation | T. Barfoot, *State Estimation for Robotics* | Matematiksel olarak en temiz anlatım; ücretsiz PDF var. |
| 📷 Geometri | R. Hartley, A. Zisserman, *Multiple View Geometry in Computer Vision* | Pinhole, epipolar, PnP. |
| 📷 VO tutorial | D. Scaramuzza, F. Fraundorfer, *Visual Odometry: Part I & II* (IEEE RAM 2011–2012) | VO'ya en iyi giriş. |
| 🤖 LightGlue | P. Lindenberger ve ark., *LightGlue: Local Feature Matching at Light Speed* (ICCV 2023) | Projedeki matcher. |
| 🤖 ALIKED | X. Zhao ve ark., *ALIKED: A Lighter Keypoint and Descriptor Extraction Network via Deformable Transformation* (2023) | Projedeki feature extractor. |
| 🎲 MAGSAC | D. Barath ve ark., *MAGSAC: Marginalizing Sample Consensus* (CVPR 2019) | Projedeki RANSAC varyantı. |
| 📏 Değerlendirme | Z. Zhang, D. Scaramuzza, *A Tutorial on Quantitative Trajectory Evaluation for Visual(-Inertial) Odometry* (IROS 2018) | ATE, RPE ve alignment'ın doğru kullanımı. |
| 📦 Veri | A. Geiger ve ark., *Vision meets Robotics: The KITTI Dataset* (IJRR 2013) | Kullandığımız veri seti. |

<div align="center">

[← Ders 6](06_evaluation_and_showcase.md) · [İçindekiler](README.md)

</div>

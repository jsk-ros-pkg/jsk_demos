import numpy
import cv2
from glob import glob
import numpy as np

#--------------------------------------------------------1.カメラそれぞれのキャリブレーション
square_size = 0.02423      # 正方形のサイズ
pattern_size = (9, 6)  # 格子数
pattern_points = numpy.zeros( (numpy.prod(pattern_size), 3), numpy.float32 ) #チェスボード（X,Y,Z）座標の指定 (Z=0)
pattern_points[:,:2] = numpy.indices(pattern_size).T.reshape(-1, 2)
pattern_points *= square_size
obj_points = []
img_points = []

#--------------------------------------------------------1-1.左カメラ
for fn in glob("calib_imgs/1/left*.jpg"):
    # 画像の取得
    im = cv2.imread(fn, 0)
    print("loading..." + fn)
    # チェスボードのコーナーを検出
    found, corner = cv2.findChessboardCorners(im, pattern_size)
    # コーナーがあれば
    if found:
        term = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT, 30, 0.1)
        cv2.cornerSubPix(im, corner, (5,5), (-1,-1), term)    #よくわからないがサブピクセル処理（小数点以下のピクセル単位まで精度を求める）
        cv2.drawChessboardCorners(im, pattern_size, corner,found)
        cv2.imshow('found corners in ' + fn,im)
    # コーナーがない場合のエラー処理
    if not found:
        print('chessboard not found')
        continue
    img_points.append(corner.reshape(-1, 2))   #appendメソッド：リストの最後に因数のオブジェクトを追加 #corner.reshape(-1, 2) : 検出したコーナーの画像内座標値(x, y)
    obj_points.append(pattern_points)
    cv2.destroyAllWindows()


# 内部パラメータを計算
rms, K_l, d_l, r, t = cv2.calibrateCamera(obj_points, img_points,(im.shape[1],im.shape[0]), None, None)
# 計算結果を表示
print("RMS = ", rms)
print("K = \n", K_l)
print("d = ", d_l.ravel())
# 計算結果を保存
numpy.savetxt("K_left.csv", K_l, delimiter =',',fmt="%0.14f") #カメラ行列の保存
numpy.savetxt("d_left.csv", d_l, delimiter =',',fmt="%0.14f") #歪み係数の保存


#--------------------------------------------------------1-2.右カメラ

obj_points = []
img_points = []


for fn in glob("calib_imgs/1/right*.jpg"):
    # 画像の取得
    im = cv2.imread(fn, 0)
    print("loading..." + fn)
    # チェスボードのコーナーを検出
    found, corner = cv2.findChessboardCorners(im, pattern_size)
    # コーナーがあれば
    if found:
        term = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT, 30, 0.1)
        cv2.cornerSubPix(im, corner, (5,5), (-1,-1), term)    #よくわからないがサブピクセル処理（小数点以下のピクセル単位まで精度を求める）
        cv2.drawChessboardCorners(im, pattern_size, corner,found)
        cv2.imshow('found corners in ' + fn,im)
    # コーナーがない場合のエラー処理
    if not found:
        print('chessboard not found')
        continue
    img_points.append(corner.reshape(-1, 2))   #appendメソッド：リストの最後に因数のオブジェクトを追加 #corner.reshape(-1, 2) : 検出したコーナーの画像内座標値(x, y)
    obj_points.append(pattern_points)
    print('found corners in ' + fn + ' is adopted')
    cv2.destroyAllWindows()


# 内部パラメータを計算
rms, K_r, d_r, r, t = cv2.calibrateCamera(obj_points,img_points,(im.shape[1],im.shape[0]), None, None)
# 計算結果を表示
print("RMS = ", rms)
print("K = \n", K_r)
print("d = ", d_r.ravel())
# 計算結果を保存
numpy.savetxt("K_right.csv", K_r, delimiter =',',fmt="%0.14f") #カメラ行列の保存
numpy.savetxt("d_right.csv", d_r, delimiter =',',fmt="%0.14f") #歪み係数の保存


#--------------------------------------------------------2.ステレオビジョンシステムのキャリブレーション
N = 29 #キャリブレーション用ステレオ画像のペア数
#　　　　「left0.jgp」のように、ペア番号を'left','right'の後につけて同じフォルダに置く(grobが使いこなせれば直したい)

square_size = 0.02423      # 正方形のサイズ
pattern_size = (9, 6)  # 格子数
pattern_points = numpy.zeros( (numpy.prod(pattern_size), 3), numpy.float32 ) #チェスボード（X,Y,Z）座標の指定 (Z=0)
pattern_points[:,:2] = numpy.indices(pattern_size).T.reshape(-1, 2)
pattern_points *= square_size
obj_points = []
img_points1 = []
img_points2 = []


for i in range(1, N + 1):
    # 画像の取得
    im_l = cv2.imread("calib_imgs/1/left" +str(i)+ ".jpg", 0)
    im_r = cv2.imread("calib_imgs/1/right" +str(i)+ ".jpg", 0)
    print("loading..." + "left" +str(i)+ ".jpg")
    print("loading..." + "right" +str(i)+ ".jpg")
    #コーナー検出
    found_l, corner_l = cv2.findChessboardCorners(im_l, pattern_size)
    found_r, corner_r = cv2.findChessboardCorners(im_r, pattern_size)
    # コーナーがあれば
    if found_l and found_r:
        term = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT, 30, 0.1)
        cv2.cornerSubPix(im_l, corner_l, (5,5), (-1,-1), term)
        cv2.cornerSubPix(im_r, corner_r, (5,5), (-1,-1), term)
        cv2.drawChessboardCorners(im_l, pattern_size, corner_l,found_l)
        cv2.drawChessboardCorners(im_r, pattern_size, corner_r,found_r)
        # cv2.imshow('found corners in ' + "left" +str(i)+ ".jpg", im_l)
        # cv2.imshow('found corners in ' + "right" +str(i)+ ".jpg", im_r)
        # cv2.waitKey(10)
    # コーナーがない場合のエラー処理
    if not found_l:
        print('chessboard not found in leftCamera')
        continue
    if not found_r:
        print('chessboard not found in rightCamera')
        continue

    # 選択ボタンを表示
    img_points1.append(corner_l.reshape(-1, 2))
    img_points2.append(corner_r.reshape(-1, 2))
    obj_points.append(pattern_points)
    print('found corners in ' + str(i) + ' is adopted')
cv2.destroyAllWindows()

# システムの外部パラメータを計算
imageSize = (im_l.shape[1],im_l.shape[0])
cameraMatrix1 = K_l
cameraMatrix2 = K_r
distCoeffs1 = d_l
distCoeffs2 = d_r
retval, cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2, R, T, E, F = cv2.stereoCalibrate(
    obj_points, img_points1, img_points2, cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2, imageSize)

# 計算結果を表示
print("retval = ", retval)
print("R = \n", R)
print("T = \n", T)
# 計算結果を保存
numpy.savetxt("cameraMatrix1.csv", cameraMatrix1, delimiter =',',fmt="%0.14f") #新しいカメラ行列を保存
numpy.savetxt("cameraMatrix2.csv", cameraMatrix2, delimiter =',',fmt="%0.14f")
numpy.savetxt("distCoeffs1.csv", distCoeffs1, delimiter =',',fmt="%0.14f") #新しい歪み係数を保存
numpy.savetxt("distCoeffs2.csv", distCoeffs2, delimiter =',',fmt="%0.14f")
numpy.savetxt("R.csv", R, delimiter =',',fmt="%0.14f") #カメラ間回転行列の保存
numpy.savetxt("T.csv", T, delimiter =',',fmt="%0.14f") #カメラ間並進ベクトルの保存


rectify_scale = 0
# T = np.array([-1, 0, 0.5]).reshape(3, 1)
R1, R2, P1, P2, Q, roi1, roi2 = cv2.stereoRectify( \
        cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2, (640, 360), R, 1000 * T, alpha=rectify_scale)
# prepare to remap the webcams
l_maps = cv2.initUndistortRectifyMap(cameraMatrix1, distCoeffs1, R1, P1, (640, 360), cv2.CV_16SC2)
r_maps = cv2.initUndistortRectifyMap(cameraMatrix2, distCoeffs2, R2, P2, (640, 360), cv2.CV_16SC2)

i = 1
lframe = cv2.imread("calib_imgs/1/left" +str(i)+ ".jpg", 1)
rframe = cv2.imread("calib_imgs/1/right" +str(i)+ ".jpg", 1)
# use the rectified data to do remap on webcams
lframe_remap = cv2.remap(lframe, l_maps[0], l_maps[1], cv2.INTER_LINEAR)
rframe_remap = cv2.remap(rframe, r_maps[0], r_maps[1], cv2.INTER_LINEAR)
viz = np.concatenate([lframe_remap, rframe_remap], axis=1)
cv2.imshow('viz', viz)
cv2.waitKey()
cv2.destroyAllWindows()

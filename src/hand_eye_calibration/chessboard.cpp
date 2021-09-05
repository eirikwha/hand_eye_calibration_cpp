//
// Created by eirik on 28.05.19.
//

#include <hand_eye_calibration/chessboard.h>

using namespace std;

ChessBoard::ChessBoard(){

    //img = readColorImage(fileName);
    numCornersHor = 9;
    numCornersVer = 7;
    squareSize = 10; // TODO: Is this mm???
    boardSize = cv::Size(numCornersHor, numCornersVer);
/*
    imgWidth = 1920;
    imgHeight = 1200;
    imgDim = cv::Size(imgWidth, imgHeight); // TODO: Private /protected variable?
*/
    //std::vector<cv::Point2f> cornersImg;
    //std::vector<cv::Point3f> corners3d;
    genCorners3d();

}

ChessBoard::ChessBoard(int &numCornersHor, int &numCornersVer, float &squareSz){
    boardSize = cv::Size(numCornersHor, numCornersVer);
    squareSize = squareSz;

    //std::vector<cv::Point3f> corners3d;
    genCorners3d();
}

void ChessBoard::genCorners3d() {
    for (int i = 0; i < boardSize.height; i++) {
        for (int j = 0; j < boardSize.width; j++) {
            corners3d.emplace_back(cv::Point3f((float) j * squareSize, (float) i * squareSize,0.0f));
        }
    } // TODO: Maybe not possible without return, might have to use void.
}

void ChessBoard::setNumCornersHor(int num){
    numCornersHor = num;
}

void ChessBoard::setNumCornersVer(int num){
    numCornersVer = num;
}

void ChessBoard::setSquareSize(int num){
    squareSize = num;
}

cv::Size ChessBoard::getBoardSize(){
    return boardSize;
}

float ChessBoard::getSquareSize(){
    return squareSize;
}
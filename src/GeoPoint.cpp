#include "GeoPoint.h"

using namespace std;

GeoPoint::GeoPoint(int id, std::string label, double longitude, double latitude) {
    this->id = id;
    this->label = label;
    this->longitude = longitude;
    this->latitude = latitude;
}

int GeoPoint::getId(){
    return this->id;
}

void GeoPoint::setId(int id){
    this->id = id;
}

std::string GeoPoint::getLabel(){
    return this->label;
}

void GeoPoint::setLabel(std::string label){
    this->label = label;
}

double GeoPoint::getLongitude(){
    return this->longitude;
}

void GeoPoint::setLongitude(double longitude){
    this->longitude = longitude;
}

double GeoPoint::getLatitude(){
    return this->latitude;
}

void GeoPoint::setLatitude(double latitude){
    this->latitude = latitude;
}

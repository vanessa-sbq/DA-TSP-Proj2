#ifndef PROJ_DA_2_GEOPOINT_H
#define PROJ_DA_2_GEOPOINT_H

#include <string>

class GeoPoint{
private:
    int id;
    std::string label;
    double longitude;
    double latitude;

public:
    GeoPoint(); // Default constructor
    GeoPoint(int id, std::string label, double longitude, double latitude);
    int getId();
    void setId(int id);
    std::string getLabel();
    void setLabel(std::string label);
    double getLongitude();
    void setLongitude(double longitude);
    double getLatitude();
    void setLatitude(double latitude);

    bool operator==(const GeoPoint& other) const {
        return (id == other.id);
    }
};


#endif //PROJ_DA_2_GEOPOINT_H

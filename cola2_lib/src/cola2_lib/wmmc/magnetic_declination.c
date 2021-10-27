/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#include "magnetic_declination.h"
#include <math.h>
#include <stdio.h>
/*#include "EGM9615.h"*/
#include "GeomagnetismHeader.h"

int computeMagneticDeclinationDeg(double *declination, const double latitude, const double longitude, const int year,
                                  const int month, const int day)
{
  /* Set magnetic declination to 0.0 so that in case of an error it has always the same value */
  *declination = 0.0;

  /* Check latitude and longitude */
  if ((latitude < -90.0) || (latitude > 90.0))
  {
    return MAGNETIC_DECLINATION_LATITUDE_OUT_OF_RANGE;
  }
  if ((longitude < -180.0) || (longitude > 180.0))
  {
    return MAGNETIC_DECLINATION_LONGITUDE_OUT_OF_RANGE;
  }

  /* Initialize structures */
  MAGtype_MagneticModel *magnetic_models[1], *timed_magnetic_model;
  MAGtype_Ellipsoid ellip;
  MAGtype_CoordSpherical coord_spherical;
  MAGtype_CoordGeodetic coord_geodetic;
  MAGtype_Date user_date;
  MAGtype_GeoMagneticElements geo_magnetic_elements;
  MAGtype_Geoid geoid;

  /* Allocate memory and read model */
  char cof_path[] = COF_DATA_PATH;
  if (!MAG_robustReadMagModels(cof_path, &magnetic_models, 1))
  {
    return MAGNETIC_DECLINATION_COF_DATA_NOT_FOUND;
  }

  /* Allocate time modified model memory */
  int nMax = 0;
  if (nMax < magnetic_models[0]->nMax)
  {
    nMax = magnetic_models[0]->nMax;
  }
  int NumTerms = ((nMax + 1) * (nMax + 2) / 2);
  timed_magnetic_model = MAG_AllocateModelMemory(NumTerms);  /* For storing the time modified WMM Model parameters */
  if ((magnetic_models[0] == NULL) || (timed_magnetic_model == NULL))
  {
    MAG_FreeMagneticModelMemory(magnetic_models[0]);
    return MAGNETIC_DECLINATION_BAD_ALLOCATION;
  }

  /* Set default values and constants */
  MAG_SetDefaults(&ellip, &geoid);

  /* Set EGM96 Geoid parameters. GeoidHeighBuffer is an array inside EGM9615.h */
  /*geoid.GeoidHeightBuffer = GeoidHeightBuffer;
  geoid.Geoid_Initialized = 1;*/

  /* Set latitude and longitude */
  coord_geodetic.phi = latitude;
  coord_geodetic.lambda = longitude;

  /* Assume request is at mean sea level*/
  /*geoid.UseGeoid = 1;
  coord_geodetic.HeightAboveGeoid = 100.0;
  MAG_ConvertGeoidToEllipsoidHeight(&coord_geodetic, &geoid);*/

  /*geoid.UseGeoid = 0;
  double h = 0;
  MAG_GetGeoidHeight(latitude, latitude, &h, &geoid);
  coord_geodetic.HeightAboveEllipsoid = (h + 100.0) * 1e-03;*/
  coord_geodetic.HeightAboveEllipsoid = 0.0;

  /* Date */
  user_date.Month = month;
  user_date.Day = day;
  user_date.Year = year;
  char error_message[255];
  if (!MAG_DateToYear(&user_date, error_message))
  {
    MAG_FreeMagneticModelMemory(timed_magnetic_model);
    MAG_FreeMagneticModelMemory(magnetic_models[0]);
    return MAGNETIC_DECLINATION_INVALID_DATE;
  }
  if ((user_date.DecimalYear > magnetic_models[0]->CoefficientFileEndDate) ||
      (user_date.DecimalYear < magnetic_models[0]->epoch))
  {
    MAG_FreeMagneticModelMemory(timed_magnetic_model);
    MAG_FreeMagneticModelMemory(magnetic_models[0]);
    return MAGNETIC_DECLINATION_INVALID_YEAR;
  }

  /* Time adjust the coefficients, Equation 19, WMM Technical report */
  MAG_TimelyModifyMagneticModel(user_date, magnetic_models[0], timed_magnetic_model);

  /*Convert from geodetic to Spherical Equations: 17-18, WMM Technical report*/
  MAG_GeodeticToSpherical(ellip, coord_geodetic, &coord_spherical);

  /* Computes the geoMagnetic field elements and their time change*/
  MAG_Geomag(ellip, coord_spherical, coord_geodetic, timed_magnetic_model, &geo_magnetic_elements);

  /* Computes the grid variation for |latitudes| > MAG_MAX_LAT_DEGREE */
  MAG_CalculateGridVariation(coord_geodetic, &geo_magnetic_elements);

  /* Free memory */
  MAG_FreeMagneticModelMemory(timed_magnetic_model);
  MAG_FreeMagneticModelMemory(magnetic_models[0]);

  /* Print debug info */
  /*printf("Latitude: %f\n", latitude);
  printf("Longitude: %f\n", longitude);
  printf("Year: %d\n", year);
  printf("Month: %d\n", month);
  printf("Day: %d\n", day);
  printf("Declination: %f\n", geo_magnetic_elements.Decl);*/

  /* Set value in degrees. Angle between the magnetic field vector and true north, positive east */
  *declination = geo_magnetic_elements.Decl;
  return MAGNETIC_DECLINATION_OK;
}

int computeMagneticDeclination(double *declination, const double latitude, const double longitude, const int year,
                               const int month, const int day)
{
  double declination_deg;
  const int error = computeMagneticDeclinationDeg(&declination_deg,
                                                  180.0 / M_PI * latitude, 180.0 / M_PI * longitude,
                                                  year, month, day);
  *declination = M_PI / 180.0 * declination_deg;
  return error;
}

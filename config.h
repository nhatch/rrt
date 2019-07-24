#ifndef CONFIG_H
#define CONFIG_H

extern const double MIN_X;
extern const double MAX_X;
extern const double MIN_Y;
extern const double MAX_Y;

struct Config {
  double x;
  double y;
  double theta;
};

Config randConfig();
void printConfig(const Config &c);
double innerProduct(const Config &c0, const Config &c1);
Config configDiff(const Config &c0, const Config &c1);
double configDist(const Config &c0, const Config &c1);
Config linComb(const Config &c0, const Config &line, double alpha);

#endif

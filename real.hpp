
// if sim is defined, dont try and run the arm from gpio
// commont out define on pi so that it runs the arm
#define SIM
// this file is outside src/ beacuse deploy.sh copies the entire src/ over and
// i dont want it to overwrite this file on the pi
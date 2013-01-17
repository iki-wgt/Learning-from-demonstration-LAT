#include <iostream>
#include <string>
#include "math.h"
#include "leatra.hh"
#include "draw.hh"
#include "litera.hh" 
#include "dtw2.hh"
#include "dtw_leatra.hh"
#include "lfd.hh"


// #define PI 3.14159265


using namespace std;


deque< ndmapSet > create_test_sets();
void print_help();
void print_error();
/*
int test1(){

  litera lit;
  ndmap map[3];
  ndmapSet set;
  ndmapSetGroup group;
   
  map[0] = lit.read_ndmap("Katana.tra", "demo1.txt");
  map[0].thinning(200);
  set.add_ndmap(map[0]);

  map[1] = lit.read_ndmap("Katana.tra", "demo2.txt");
  map[1].thinning(200);
  set.add_ndmap(map[1]);

  map[2] = lit.read_ndmap("Katana.tra", "demo3.txt");
  map[2].thinning(200);
  set.add_ndmap(map[2]);

  group.add_ndmapSet(set);

//  lit.write_ndmap(map);  
  
  
  lit.write_all(group, "Katana.tra/");
  
  
  int origin = map.map_is_consistent();
  
  for(int i = 0; i < origin; i++){
    
    map.thinning(origin - i);
  
    map.print();
    
    char c;
    cin >> c;
  
  }

  return 1;
}

int test3(){

  litera lit;
  ndmap map[3];
  ndmapSet set;
  ndmapSetGroup group1, group2;
   
  map[0] = lit.read_ndmap("Katana.tra", "demo1.txt");
  map[0].thinning(200);
 // map[0].map[0] = map[0].map[5];
 // map[0].map.resize(1);
  set.add_ndmap(map[0]);
  
  map[1] = lit.read_ndmap("Katana.tra", "demo2.txt");
  map[1].thinning(200);
//  map[1].map[0] = map[1].map[5];
 // map[1].map.resize(1);
  set.add_ndmap(map[1]);
  
  //set.smoothing(1);
  
  map[2] = lit.read_ndmap("Katana.tra", "demo3.txt");
  map[2].thinning(200);
//  map[2].map[0] = map[1].map[5];
//  map[2].map.resize(1);
//  set.add_ndmap(map[2]);

  group1.add_ndmapSet(set);
  
 // warp_master warp, warp1, warp2;
 // warp.warp_set_nD( &set );
  
  group2.add_ndmapSet(set);
  
  lit.write_all(group1, "VarpNo1/");
  lit.write_all(group2, "VarpYes1/");
  

  approximation apx;
  ndmap mean = apx.mean_value(set);
  
  mean.set_name("DEMO3");
  lit.write_ndmap(mean);  
 
  int origin = map.map_is_consistent();
  
  for(int i = 0; i < origin; i++){
    
    map.thinning(origin - i);
  
    map.print();
    
    char c;
    cin >> c;
  
  }

  return 1;
}

int test4(){

  litera lit;
  ndmap map[3];
  ndmapSet set;
  ndmapSetGroup group1, group2;
   
  map[0] = lit.read_ndmap("Katana.tra", "demo1.txt");
  map[0].thinning(200);
  // map[0].map[0] = map[0].map[5];
  // map[0].map.resize(1);
  set.add_ndmap(map[0]);
  
  map[1] = lit.read_ndmap("Katana.tra", "demo3.txt");
  map[1].thinning(200);
  //  map[1].map[0] = map[1].map[5];
  // map[1].map.resize(1);
  set.add_ndmap(map[1]);
 
  // Entfernen aller Dimensionen
  for(int i = set.get_dim() - 1 ; i > 0 ; i--)
    set.remove_dimension(i);
  
  
}


int test_simple_warping_test(){

  litera lit;
  ndmap map[2], mapA, mapB, mapC, mapD;
  ndmapSet set1, set2;
  ndmapSetGroup group1, group2;
   
  map[0] = lit.read_ndmap("Katana.tra", "demo1.txt");
  map[0].thinning(400);
  // map[0].map[0] = map[0].map[5];
  // map[0].map.resize(1);
  std::deque< double > x = map[0].get_row(0);
  
  map[1] = lit.read_ndmap("Katana.tra", "demo3.txt");
  map[1].thinning(400);
  // map[1].map[0] = map[1].map[5];
  // map[1].map.resize(1);
  
  std::deque< double > y = map[1].get_row(0);
  
  ndmap mapx, mapy;
  
  mapx.add_deque( x );
  mapy.add_deque( y );
  
  trajectory_lat trajectory1, trajectory2;
  trajectory1.set_ndmap(mapx);
  trajectory2.set_ndmap(mapy);
 // trajectory1.invert();
 // trajectory2.invert();
  std::deque< trajectory_lat > tra;
  
  tra.push_back(trajectory1);
  tra.push_back(trajectory2);

  mapC.add_deque(tra[0].get_ndmap().get_row(0)); 
  mapD.add_deque(tra[1].get_ndmap().get_row(0)); 

  set1.add_ndmap(mapC);
  set1.add_ndmap(mapD);

  warp_leatra W;
  W.warp_trajectories(&tra);
  
  //tra[0].invert();
 // tra[1].invert();
  
  mapA.add_deque(tra[0].get_ndmap().get_row(0)); 
  mapB.add_deque(tra[1].get_ndmap().get_row(0)); 
  
  set2.add_ndmap(mapA);
  set2.add_ndmap(mapB);
  
  
  group1.add_ndmapSet(set1);
  group2.add_ndmapSet(set2);
  
  lit.write_all(group1, "VarpNo2/");
  lit.write_all(group2, "VarpYes2/");
  

  approximation apx;
  ndmap mean = apx.mean_value(set);
  
  mean.set_name("DEMO3");
  lit.write_ndmap(mean);  
 
  int origin = map.map_is_consistent();
  
  for(int i = 0; i < origin; i++){
    
    map.thinning(origin - i);
  
    map.print();
    
    char c;
    cin >> c;
  
  }

  return 1;
}
*/


int test(){

  std::deque< object > objs;
  object obj1, obj2;
  obj1.set_name("IKEA-CUP-SOLBRAEND-BLUE");
  obj1.add_coordinate(363.458);
  obj1.add_coordinate(121.549);
  obj1.add_coordinate(643.472);
  
  obj2.set_name("COCA-COLA-CAN-250ML");  
  obj2.add_coordinate(448.412);
  obj2.add_coordinate(-85.2141);
  obj2.add_coordinate(655.665);
  
  objs.push_back(obj1);
  objs.push_back(obj2);
  
  lfd LaT;
  
  std::deque< std::deque< double > > trajectory = LaT.reproduce( objs, "task1");
  
  ndmap map;
  map.set_deque(trajectory);
  map.set_name("test2");
  
  litera lit;
  std::cout << " Hello I am here!!!!" << std::endl;
  lit.write_ndmap(map);
  
  return 1;
}


 // coordinate[0] = coordinate[0] - 38;
 // coordinate[1] = coordinate[1] - 0;
 // coordinate[2] = coordinate[2] - 592;s

// 2.88054 0.926767 1.76516 2.7656 3.15999 -2.05943 
// 2.88202 -0.213424 1.56418 1.50712 3.15606 -2.05943 
/*
int DK_IK_test(){

  std::deque< double > angles;
  angles.push_back(2.88202);
  angles.push_back(-0.213424);
  angles.push_back(1.56418);
  angles.push_back(1.50712);
  angles.push_back(3.15606);
    
  std::deque< double > xyz = DK_lat(angles);
  
    std::cout<<"DK IK TEST: " << std::endl;
  std::cout<<"Winkel 1     = ";
  for(int i = 0; i < 5; i++){
    std::cout << angles[i] << "; ";
  }
  std::cout<< std::endl;
  std::cout<<"Koordinaten 1 = ";
  for(int i = 0; i < 3; i++){
    std::cout << xyz[i] << "; ";
  }
  std::cout<< std::endl;

  
  for(int i = 0; i < 5; i++){
    angles[i] = angles[i] - 4;
  }
  std::deque< double > xyz_neu = DK_lat(angles);
  
  std::cout<<" " << std::endl;
  std::cout<<"Winkel 2      = ";
  for(int i = 0; i < 5; i++){
    std::cout << angles[i] << "; ";
  }
  std::cout<< std::endl;
  std::cout<<"Koordinaten 2 = ";
  for(int i = 0; i < 3; i++){
    std::cout << xyz_neu[i] << "; ";
  }
  std::cout<< std::endl;
  
  
  std::deque< double > new_angle = angles;
  for(int i = 0; i < 10; i++){
    new_angle = IK_lat(new_angle, DK_lat(new_angle), xyz);
  } 
  
  std::deque< double > xyz2 = DK_lat(new_angle);
  
  std::cout<<"\nKoordinaten neu = ";
  for(int i = 0; i < 3; i++){
    std::cout << xyz2[i] << "; ";
  }
  std::cout<< std::endl;
}

*/




/**
 *  The main function provides the terminal user with some basic sample functions of LEATRA.
 */

int main( int argc, const char* argv[] ){
  
  init();  
  ndmapSet set;
  ndmapSetGroup model;
  approximation apx;
  draw pic;
  litera lit;
  
  int level = 0;
  int command = 0;
  
  for( int i = 1; i < argc; i++ ){
    
    string input(argv[i]);
    
    switch(level){
    
      case 0: 
             if( input == "-h" ) {command = 1; level = 8; }
        else if( input == "-dt") {command = 2; level = 1; }
        else if( input == "-rm") {command = 2; level = 2; }
        else if( input == "-t" ) {command = 10;level = 8; }
        else command = -1;
        break;
        
      case 1: 
        command = 3; level = 3;
        break;

      case 2: 
        command = 4; level = 4;
        break;
        
      case 3: 
        level = 4;
        if( input == "-sm") {command = 5; break;}
        continue;
      case 4:
        if( input == "-r" ) {command = 2; level = 5; }
        else command = -1;
        break;
    
      case 5:
        command = 6; level = 6;  
        break;
      
      case 6: 
             if( input == "-p" ) {command = 7; level = 7; }
        else if( input == "-st") {command = 8; level = 7; }
        else command = -1;
        break;
    
      case 7: 
             if( input == "-p" ) {command = 7; level = 8; }
        else if( input == "-st") {command = 8; level = 8; }
        else command = -1;
        break;
      
      case 8:
          return 0;
      
    }
    try{
        
      switch(command){
        
        case 1: // print help for usage
          print_help();
          break;
      
        case 2: // nothing to do (wait for next input) ...
          break; 
      
        case 3: // read trajectory from file and make model
          model = apx.make_model(lit.read_trajectories( input ));
          model.set_name( input );
          break;
      
        case 4: // read model from file
          model = lit.read_group( input );
          break;
        
        case 5: // safe model to file
          lit.write_ndmapSetGroup( model );
          break;
      
        case 6: // read object file and perform reproduction of a new trajectory
          model.add_offset(lit.read_objects( input ));
          set = apx.constraint_fusion( model );
          model.add_ndmapSet( set );
          break;
      
        case 7: // print a pdf file
          pic.graph_std( model );
          break;
        
        case 8: // save the calculated trajectory (ndmap)
          lit.write_ndmap( set.get_ndmap(1) );
          break;
   
        case 10: // run test function
          test();
          break;
  
        default: // print error information and how to get help
          print_error();
          return 0;
      }    
    }
    catch(data_error& err){
      return err.print();
    }
  }
  return 0;
}


/**
 *  The print_help function provides the terminal user with the information necessary in order to run the terminal samples.
 */
void print_help(){
    cout<<"\nLeatra, version 1\n";
    cout<<"The command line program of Leatra gives insight in the fuctionality of Leatra, but supports only a limited number of options.\n";
    cout<<"The Leatra project has been initialized in order to implement a mathematical solution for a \n";
    cout<<"Learning from demonstrations (Lfd) method - an eager learning technique. \n";
    cout<<"For further information, please see the documentation at: leatra/src/html/index.html and leatra/documentation/leatra_documentation.pdf .\n\n";              
    cout<<"Usage: ./leatra [-dt [optionally -sm] or -rm] -r [-p or -st] \n\n";
    cout<<"\t -dt : \"demonstration trajectories\": read trajectory data from file, must be followed by the name of the subfolder(***)\n";
    cout<<"\t -rm : \"read model\": read a model from file, must be followed by the name of the subfolder(**)\n";
    cout<<"\t -sm : \"save model\": save the produced model to a file\n";
    cout<<"\t -r  : \"reproduction\": find a trajectory for the before defined model.\n";
    cout<<"\t       This parameter needs to be followed by the name of an object file (****)\n";
    cout<<"\t -p  : \"print\": plots a pdf of the results\n";
    cout<<"\t -st : \"save trajectory\": saves the calculated trajectory to a data file*.\n\n";      
    cout<<"\t Explanations:\n";   
    cout<<"\t *   : Data file\n"; 
    cout<<"\t       A data file contains a multidimensional trajectory in Leatra.\n"; 
    cout<<"\t       Each column contains the data of one dimension. \n";
    cout<<"\t       The columns need to be separated by a space or a tab. \n"; 
    cout<<"\t       All columns need to be of the same size.\n";
    cout<<"\t **  : Model file\n";  
    cout<<"\t       The model file is produced automatically by Leatra, when saving a model to file.\n";
    cout<<"\t       As a model consists out of information regarding several objects, again several trajectories are needed.\n";
    cout<<"\t       The model for one object consists out of the following trajectories:\n";
    cout<<"\t       1. mean plus standard deviation (mean+stdev) \n"; 
    cout<<"\t       2. mean  \n"; 
    cout<<"\t       3. mean minus standard deviation (mean+stdev) \n"; 
    cout<<"\t       4. standard deviation (stdev) \n"; 
    cout<<"\t       The order of the files needs to be observed. Example file coffee.mod:\n";
    cout<<"\t                  Cup   \n";
    cout<<"\t                  Cup_mean+stdev;Cup_mean;Cup_mean-stdev;Cup_stdev;   \n";
    cout<<"\t                  Coffee machine   \n";
    cout<<"\t                  CM_mean+stdev;CM_mean;CM_mean-stdev;CM_stdev;   \n";
    cout<<"\t       The two models for the Cup and the Coffee machine are defined by the four saved trajectories\n";
    cout<<"\t       mentioned above, ended by a semicolon.\n";
    cout<<"\t       This file needs to be placed in a subfolder in the 'leatra' directory with the same name \n";
    cout<<"\t       like the model file (also ending with .mod), together with the data files.\n";
    cout<<"\t *** : Trajectory file \n";
    cout<<"\t       In order to make the involved objects known to Leatra, the trajectory file is needed.\n";   
    cout<<"\t       For example, if there have been two demonstrations (demo1 and demo2), \n";   
    cout<<"\t       Leatra needs to know all coordinates of the involved objects at all demonstrations.\n";   
    cout<<"\t       The trajectory file needs do have the ending '.tra' and needs to be structured as follows: \n";   
    cout<<"\t       Alternating, one line contains the name of the data file (a trajectory),  \n";  
    cout<<"\t       the following line contains the names of the objects, followed in brackets their coordinates.\n"; 
    cout<<"\t       Example file make coffee.tra:\n"; 
    cout<<"\t                  demo1\n";
    cout<<"\t                  Cup(1.49, 3.45)Coffee_maker(34.5, 44.3)\n";
    cout<<"\t                  demo2\n";
    cout<<"\t                  Cup(5.2, 43.01)Coffee_maker(56.5, 13.7)\n";
    cout<<"\t       Note that there are no whitespaces allowed anywhere! This file needs to be placed in a subfolder \n";
    cout<<"\t       in the 'leatra' directory carrying the same name like the trajectory file (also with the ending .tra),\n";
    cout<<"\t       together with the data files\n";
    cout<<"\t ****: The object file has only one line as follows: \n";
    cout<<"\t                  Cup(23.0, 25.3)Coffee_maker(5.5, 7)\n";    
    cout<<"\t       Author: Heiko Posenauer 2012 \n";
}

/**
 *  The print_error function prints an error to the terminal, when there are syntactical errors, while running the terminal samples.
 */
void print_error(){
    cout<<"LEATRA Error: arguments have not been recognized. For help please use the \"-h\" key!\n";
}

/**
 *  The create_test_group creates a sample test group in order to get to know the functionality, even though the user didn't set up data files.
 */
deque< ndmapSet > create_test_sets(){

  deque< double > deq[6];
  for(int i=0; i<600; i++){
    deq[0].push_back(sin((((double)i * PI)/180.))- 2 + ((double)i)/600.0);
    deq[1].push_back(cos(((double)i * PI)/360.)-2);
    deq[2].push_back((80. - i)/ 80);
    deq[3].push_back(((120. - i)/160.) + 2);   
    deq[4].push_back(((30. - i)/160.) + 2);   
    deq[5].push_back(((400. - i)/150.) + 2);   
  }
  
  ndmap ND[6];
  for(int m=0; m < 6; m++) {
    for(int d=0; d < 8; d++) ND[m].add_deque(deq[m]);   
    ND[m].set_name("f" + intTOstring(m));
    ND[m].correct_nans();
  }
  
  ndmapSet SET1, SET2;
  for(int s=0; s < 3; s++){
    SET1.add_ndmap(ND[s]);
    SET2.add_ndmap(ND[s + 3]);
  }
  
  SET1.set_name("SET1");
  SET2.set_name("SET2");  
  
  approximation apx;
  deque< ndmapSet > sets;
  sets.push_back(apx.standard_deviation(SET1,true));
  sets.push_back(apx.standard_deviation(SET2,true));
  
  return sets;
}


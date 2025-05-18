#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "TCanvas.h"
#include "TGraph.h"
#include "TAxis.h"

void analyze()
{
  std::ifstream file("sample_data.txt");
  float time;
  float average_dist;
  float dev_dist;
  float average_vel;
  float dev_vel;
  std::vector<float> times;
  std::vector<float> average_distances;
  std::vector<float> average_velocities;
  std::string line;
  std::getline(file, line);
  while (std::getline(file, line)) {
    std::istringstream stream(line);
    if (stream >> time >> average_dist >> dev_dist >> average_vel >> dev_vel) {
      times.push_back(time);
      average_distances.push_back(average_dist);
      average_velocities.push_back(average_vel);
    } else {
      std::cout << "Error in " << line << '\n';
    }
  }
  TGraph* graph_speed =
      new TGraph(times.size(), times.data(), average_velocities.data());
  graph_speed->SetTitle("Media della velocità vs Tempo");
  graph_speed->GetXaxis()->SetTitle("Tempo (s)");
  graph_speed->GetYaxis()->SetTitle("Velocità Media");
  graph_speed->SetMarkerStyle(20);
  graph_speed->SetMarkerColor(kBlue);
  TGraph* graph_distance =
      new TGraph(times.size(), times.data(), average_distances.data());
  graph_distance->SetTitle("Media della distanza vs Tempo");
  graph_distance->GetXaxis()->SetTitle("Tempo (s)");
  graph_distance->GetYaxis()->SetTitle("Distanza Media");
  graph_distance->SetMarkerStyle(21);
  graph_distance->SetMarkerColor(kRed);
  TCanvas* c1 = new TCanvas("c1", "Mean Distance vs Time", 800, 600);
  graph_distance->Draw("APL");
  TCanvas* c2 = new TCanvas("c2", "Mean Speed vs Time", 800, 600);
  graph_speed->Draw("APL");
  c1->Update();
  c2->Update();
}
from doctest import script_from_examples
from itertools import islice
from operator import truediv, itemgetter
import collections
import operator
import json
import sys
import ROOT
from csv import DictWriter as csvDictWriter
import os
from array import array
import numpy

class Process_results():

    #set some root options
    ROOT.gStyle.SetTitleYOffset(2.20)
    ROOT.gStyle.SetTitleXOffset(2.20)
    ROOT.gROOT.SetBatch(ROOT.kTRUE)

    hybrid_serial = '_hybrid22_coldtest_m40'
    results_file = ''
    results_folder = ''
    scpi_errors = {}
    reading_header = True
    hybrid = ""
    maxLoad = ""
    step = ""
    tests = {}
    
    def __init__(self,serial_number,folder):
        self.hybrid_serial = serial_number
        self.results_folder = folder
        print(folder+'/result{}.txt'.format(self.hybrid_serial),'r')
        self.results_file = open(folder+'/result{}.txt'.format(self.hybrid_serial),'r')
        os.system('mkdir {}/results{} {}/canvas{}'.format(folder,serial_number,folder,serial_number))
        self.tests = {}

    
    def next_line(self,file):
            while True:
                line = file.readline()

                if line == "":
                    return "EOF", "EOF"
                line = line.strip()
                if line == "":
                    continue
                # print("{}".format(line))
                key, value = line.split(';')

                if key == "SCPI ERROR":
                    err_k, err_v = value.split(',')
                    self.scpi_errors[err_k] = err_v
                    continue
                return key.strip(), value.strip()

    def read_results (self):

       

        print("Setup Parameters:")

        # read first line
        k, hybrid = self.next_line(self.results_file)
        print("{} --> {}".format(k, hybrid))
        if k != "Hybrid":
            print("Expected Hybrid got instead {}".format(k))
        # read 2nd line
        k, maxLoad = self.next_line(self.results_file)
        print("{} --> {}".format(k, maxLoad))
        if k != "maxLoad":
            print("Expected maxLoad got instead {}".format(k))
        #
        k, step = self.next_line(self.results_file)
        print("{} --> {}".format(k, step))
        if k != "step":
            print("Expected step got instead {}".format(k))

        while self.reading_header:
            k, v = self.next_line(self.results_file)
            if k == "header-end" and v == "0":
                print("Full header read")
                self.reading_header = False
            print("{} --> {}".format(k, v))
        print("---------------------------------------")

        #print("Setup parameters: hybrid: {}, maxLoad: {}, step: {}".format(hybrid, maxLoad, step))

        
        load = 0
        vin = 0
        k, v = self.next_line(self.results_file)
        if k == "SET:VIN":
            vin = v
            self.tests[vin] = {}
        else:
            print("Expected VIN got {}".format(k))
            # break
            # sys.exit(1)
        k, v = self.next_line(self.results_file)

        if k != "HIV" and v != "ON":
            print("Expected initial HIV;ON, got instead {}={}".format(k, v))
            # break
            # sys.exit(1)
        # Process results until end of file

        while True:
            k, v = self.next_line(self.results_file)
            if k == "EOF" and v == "EOF":
                break
            if k == "Turning off" and v == "0":
                break
            if k == "SET:VIN":
                vin_prev = vin
                vin = v
                #load = 0
                #print("Processed vin {}, next voltage {}".format(vin_prev, vin))
                if vin not in self.tests:
                    self.tests[vin] = {}
                continue
            if k == "SET:LOAD":
                prev_load = load
                load = v
                #print("Processed load {}, next cycle load {}".format(prev_load, load))
                load = int(float(load) * 100)
                if load not in self.tests[vin]:
                    self.tests[vin][load] = {}
                # continue
            if k == "HIV:ON":
                continue

            if vin in self.tests:
                if load in self.tests[vin]:
                    if k in self.tests[vin][load]:
                        print("Duplicate key for test at load {}: {}={}".format(load, k, v))
                    self.tests[vin][load][k] = v
            # END OF PSPOH process output

        if bool(self.scpi_errors):
            print(self.scpi_errors)
        #print(json.dumps(tests, sort_keys=False, indent=4))
        print(self.tests.keys())
        print(self.tests["12"].keys())
        print(self.tests["12"][5].keys())
        print(self.tests["12"][5]['PTAT_offset'])

    def write_csv(self):
        with open('PSPOH-longtest_{}.csv'.format(self.hybrid_serial), 'w') as f:
            writer = csvDictWriter(f, fieldnames=self.tests["12"][5].keys(),delimiter=" ")
            for vin in self.tests:
                for load in self.tests[vin]:
                    writer.writerow(self.tests[vin][load])



    def get_curves(self,  meas):
        index=5
        canvas = ROOT.TCanvas()
        output_graph = ROOT.TMultiGraph()
        output_2Dgraph = ROOT.TGraph2D()
        legend = ROOT.TLegend(0.8,0.7,0.9,0.4)

        #weird ordering of the Vin solved here
        sorted_x = sorted(self.tests.items(), key=operator.itemgetter(0))
        sorted_dict = collections.OrderedDict(sorted_x)

        for vin, loads in sorted_dict.items():
            curve = ROOT.TGraph()
            curve.SetLineColor(index)
            curve.SetLineWidth(2)
            legend.AddEntry(curve,"{}".format(vin),"l"); 
            index+=1
            for load, values in loads.items():
                #if(load<20):continue
                if not meas in values:
                    print("Vin {} with load {} measurement {} not found".format(
                        vin, load, meas))
                    continue
                else:
                    #print(float(values[meas]))
                    curve.SetPoint(curve.GetN(), float(load), float(values[meas]))
                    output_2Dgraph.SetPoint(output_2Dgraph.GetN(),float(vin), float(load), float(values[meas]))
            curve.SaveAs("results{}/curve_{}_{}_{}.root".format(self.hybrid_serial, meas, str("%.1f" % float(vin)),self.hybrid_serial))
            output_graph.Add(curve)
        
        output_graph.Draw("AC")
        legend.Draw()
        canvas.SaveAs("canvas{}/canvas_{}_{}.pdf".format(self.hybrid_serial,meas,self.hybrid_serial))

        canvas2D = ROOT.TCanvas()
        output_2Dgraph.SetTitle("{} {};{};{};{}".format(self.hybrid_serial,meas,"vin","Load in %",meas))
        output_2Dgraph.Draw("surf1 ")
        #output_2Dgraph.Draw("same P0")
        output_2Dgraph.SaveAs("results{}/curve_{}_{}.root".format(self.hybrid_serial, meas,self.hybrid_serial))
        canvas2D.SaveAs("canvas{}/canvas2D_{}_{}.pdf".format(self.hybrid_serial, meas,self.hybrid_serial))


class Mean_curve():
    version_number = 1
    hybrid_list = []
    voltage_range = [10,12]
    voltage_step = 0.5
    voltage_points = int(((voltage_range[1]-voltage_range[0])/voltage_step) + 1)
    load_range = [0,120]
    load_step = 5
    load_points = int(((load_range[1]-load_range[0])/load_step) + 1)
    print("Mean curve inputs: \n" + "Voltage range: {}, voltage step: {}, voltage points: {}".format(str(voltage_range), str(voltage_step), str(voltage_points)))
    print("Load range: {}, load step: {}, load points: {}".format(str(load_range), str(load_step), str(load_points)))
    acceptance_dict = { "all" : { 0 : [10,10]},
                            "EFF_%": { 0 : [5,5],
                                    20 : [3,3] 
                                        }                         
                            }
    
    
    def __init__(self,hybrid_list):
        self.hybrid_list = hybrid_list
        print('Includes {}'.format(self.hybrid_list))

    def set_voltage_range(self,minV,maxV):
        self.voltage_range=[minV,maxV]
        self.calculate_V_Npoints()

    def set_voltage_step(self,stepV):
        self.voltage_step=stepV
        self.calculate_V_Npoints()

    def set_load_range(self,minL,maxL):
        self.load=[minL,maxL]
        self.calculate_L_Npoints()

    def set_load_step(self,stepL):
        self.load_step=stepL
        self.calculate_L_Npoints()

    def calculate_V_Npoints(self):
        self.voltage_points = int(((self.voltage_range[1]-self.voltage_range[0])/self.voltage_step) + 1)

    def calculate_L_Npoints(self):
        self.load_points = int(((self.load_range[1]-self.load_range[0])/self.load_step) + 1)
    
    def set_acceptance_range(self,dict_acc):
        self.acceptance_dict=dict_acc

    def get_acceptance_range(self,load,meas):
        range_low = 0
        range_high = 0
        for meas_key in self.acceptance_dict:
            if(meas==meas_key):
                for load_key in self.acceptance_dict[meas_key]:
                    if load_key<=load: 
                        range_low=self.acceptance_dict[meas_key][load_key][0]
                        range_high=self.acceptance_dict[meas_key][load_key][1]
                return [range_low, range_high]
            
        return [self.acceptance_dict["all"][0][0],self.acceptance_dict["all"][0][1]]

    def get_mean_curve(self, meas):
    
    
        output_graph = ROOT.TMultiGraph()
        output_dispersion = ROOT.TMultiGraph()
        output_2DGraph = ROOT.TGraph2D()

        os.system('mkdir reference_curves')
        output_csv_file = open("reference_curves/{}_ref.csv".format(meas),"w")
        output_csv_file.write("V_IN_"+meas.upper()+", LOAD_"+meas.upper()+", REF_"+meas.upper()+", LOW_%_"+meas.upper()+", HIGH_%_"+meas.upper()+", POH_ACC_{}_VERSION".format(meas.upper())+"\n")
        
        zeros = array( 'd' )
        for i in range(self.load_points):
            zeros.append( 0. )

        for vin in numpy.linspace(self.voltage_range[0], self.voltage_range[1], self.voltage_points):
            graph_content_y, graph_content_x = array( 'd' ), array( 'd' )
            graph_min_y, graph_max_y,graph_vin = array( 'd' ), array( 'd' ), array( 'd' )
            band_acceptance_low = array( 'd' )
            band_acceptance_high = array( 'd' )
            for i in range(self.load_points):
                graph_content_x.append( self.load_step*i )
                graph_content_y.append( 0. )
                graph_min_y.append( 0. )
                graph_max_y.append( 0. )
                graph_vin.append( float(vin)*100. )

            for i,hybrid in enumerate(self.hybrid_list):
                file= ROOT.TFile("results"+hybrid+"/curve_"+meas+"_"+str("%.1f" % vin)+"_"+hybrid+".root")
                curve = file.Get(";1")

                for k in range(self.load_points):
                    x,y= ROOT.Double(0), ROOT.Double(0)
                    curve.GetPoint(k,x,y)
                    graph_content_y[k]+=y
                    if(i==0):
                        graph_min_y[k]=y
                        graph_max_y[k]=y
                    else :
                        graph_min_y[k]=min(graph_min_y[k],y)
                        graph_max_y[k]=max(graph_max_y[k],y)
                        

                    
            for k in range(self.load_points):
                graph_content_y[k]=graph_content_y[k]/len(self.hybrid_list)
            for k in range(self.load_points):
                graph_min_y[k]=100.*abs(graph_min_y[k]-graph_content_y[k])/graph_content_y[k]
                graph_max_y[k]=100.*abs(graph_max_y[k]-graph_content_y[k])/graph_content_y[k]
            for k in range(self.load_points):
                output_2DGraph.SetPoint(output_2DGraph.GetN(),float(vin), float(graph_content_x[k]), float(graph_content_y[k]))

                acceptance_range = self.get_acceptance_range(float(graph_content_x[k]),meas)
                output_csv_file.write(str(vin)+", "+str(graph_content_x[k])+", "+str(graph_content_y[k])+", "+str(acceptance_range[0])+", "+str(acceptance_range[1])+", POH_ACC_{}_VERSION_".format(meas.upper())+str(self.version_number)+"\n")

                band_acceptance_low.append(acceptance_range[0])
                band_acceptance_high.append(acceptance_range[1])

            dispersion=ROOT.TGraphAsymmErrors(self.load_points,graph_content_x,graph_vin,zeros,zeros,graph_min_y,graph_max_y)
            acceptance_band=ROOT.TGraphAsymmErrors(self.load_points,graph_content_x,graph_vin,zeros,zeros,band_acceptance_low,band_acceptance_high)
            curve_for_vin = ROOT.TGraph(self.load_points,graph_content_x,graph_content_y)
            curve_for_vin.SetLineColor(1)

            output_graph.Add(curve_for_vin)
            
            dispersion.SetFillColorAlpha(ROOT.kYellow, 1.0) 
            acceptance_band.SetFillColorAlpha(ROOT.kGreen, 0.5)
            output_dispersion.Add(acceptance_band)
            output_dispersion.Add(dispersion)
            
        
        canvas = ROOT.TCanvas()
        output_graph.Draw("ALC")
        canvas.SaveAs("mean1D{}.pdf".format(meas))

        canvas_dispersion = ROOT.TCanvas()
        output_dispersion.Draw("A E3")
        output_dispersion.SetTitle("{};load;Vin*100".format(meas))
        output_dispersion.GetXaxis().SetTitleOffset(1)
        output_dispersion.GetYaxis().SetTitleOffset(1)
        canvas_dispersion.SaveAs("dispersion1D{}.pdf".format(meas))


        canvas2D = ROOT.TCanvas()
        output_2DGraph.SetTitle(" {};{};{};{}_{}".format(meas,"vin","Load in %","Ref",meas))
        output_2DGraph.Draw("surf1")
        canvas2D.SaveAs("mean2D{}.pdf".format(meas))
        output_2DGraph.SaveAs("mean2D{}.root".format(meas))
        
        output_csv_file.close()




essai_process = Process_results('PSPOH-301000003','.')
essai_process.read_results()
essai_process.get_curves("V_2v55")
essai_process.get_curves("EFF_%")
essai_process.get_curves("C_HIV")
essai_process.get_curves("T_PTAT")
essai_process.get_curves("T_PCB")
essai_process.get_curves("PTAT_offset")
essai_process.get_curves("R_2v55")
essai_process.get_curves("R_1V")
essai_process.get_curves("V_2v55")
essai_process.get_curves("V_1v25_L")
essai_process.get_curves("V_1v_L")
essai_process.get_curves("V_1v25_R")
essai_process.get_curves("V_1v_R")
essai_process.get_curves("V_1v25_T")
essai_process.get_curves("R_1V25")
"""

essai_process = Process_results('_hybrid22_coldtest_m40','.')
essai_process.read_results()
essai_process.get_curves("V_2v55")

essai_process = Process_results('PSPOH-301000022','.')
essai_process.read_results()
essai_process.get_curves("EFF_%")
"""

input_hybrids = ['PSPOH-301000004','PSPOH-301000022','PSPOH-301000003']#,'PSPOH-301000010']
essai_mean_curve = Mean_curve(input_hybrids)
essai_mean_curve.set_voltage_range(10,12)
acceptance_dict = { "all" : { 0 : [10,10]},
                            "EFF_%": { 0 : [5,5],
                                    20 : [3,3] 
                                        }                         
                            }
essai_mean_curve.set_acceptance_range(acceptance_dict)
print(essai_mean_curve.voltage_range)
print(essai_mean_curve.voltage_points)


essai_mean_curve.get_mean_curve("EFF_%")

"""
essai_mean_curve.get_mean_curve("C_HIV")
#essai_mean_curve.get_mean_curve("T_PTAT")
#essai_mean_curve.get_mean_curve("T_PCB")
#essai_mean_curve.get_mean_curve("PTAT_offset")
essai_mean_curve.get_mean_curve("R_2v55")
essai_mean_curve.get_mean_curve("R_1V")
essai_mean_curve.get_mean_curve("V_2v55")
essai_mean_curve.get_mean_curve("V_2v55")
essai_mean_curve.get_mean_curve("V_1v25_L")
essai_mean_curve.get_mean_curve("V_1v_L")
essai_mean_curve.get_mean_curve("V_1v25_R")
essai_mean_curve.get_mean_curve("V_1v_R")
essai_mean_curve.get_mean_curve("V_1v25_T")
essai_mean_curve.get_mean_curve("R_1V25")
"""
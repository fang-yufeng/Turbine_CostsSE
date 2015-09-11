"""
turbine_costsse_2015.py

Created by Janine Freeman 2015 based on turbine_costsse.py 2012.
Copyright (c) NREL. All rights reserved.
"""

from openmdao.core import Component, Problem, Group
import numpy as np

###### Rotor
#-------------------------------------------------------------------------------
class BladeCost2015(Component):

    def __init__(self):

        super(BladeCost2015, self).__init__()

        # variables
        self.add_param('blade_mass', 0.0, desc='component mass [kg]')
        self.add_param('blade_mass_cost_coeff', 13.08, desc='blade mass-cost coeff [$/kg]') #mass-cost coeff with default from ppt
    
        # Outputs
        self.add_output('blade_cost', 0.0, desc='Overall wind turbine component capital costs excluding transportation costs')

    def solve_nonlinear(self, params, unknowns, resids):

        blade_mass = params['blade_mass']
        blade_mass_cost_coeff = params['blade_mass_cost_coeff']

        # calculate component cost
        BladeCost2015 = blade_mass_cost_coeff * blade_mass
        unknowns['blade_cost'] = BladeCost2015
        

# -----------------------------------------------------------------------------------------------
class HubCost2015(Component):

    def __init__(self):

        super(HubCost2015, self).__init__()

        # variables
        self.add_param('hub_mass', 0.0, desc='component mass [kg]')
        self.add_param('hub_mass_cost_coeff', 3.80, desc='hub mass-cost coeff [$/kg]')
    
        # Outputs
        self.add_output('hub_cost', 0.0, desc='Overall wind turbine component capial costs excluding transportation costs')

    def solve_nonlinear(self, params, unknowns, resids):

        hub_mass_cost_coeff = params['hub_mass_cost_coeff']
        hub_mass = params['hub_mass']

        # calculate component cost
        HubCost2015 = hub_mass_cost_coeff * hub_mass
        unknowns['hub_cost'] = HubCost2015
        

#-------------------------------------------------------------------------------
class PitchSystemCost2015(Component):

    def __init__(self):

        super(PitchSystemCost2015,self).__init__()

        # variables
        self.add_param('pitch_system_mass', 0.0, desc='component mass [kg]')
        self.add_param('pitch_system_mass_cost_coeff', 22.91, desc='pitch system mass-cost coeff [$/kg]') #mass-cost coeff with default from list
    
        # Outputs
        self.add_output('pitch_cost', 0.0, desc='Overall wind turbine component capial costs excluding transportation costs')

    def solve_nonlinear(self, params, unknowns, resids):
        
        pitch_system_mass = params['pitch_system_mass']
        pitch_system_mass_cost_coeff = params['pitch_system_mass_cost_coeff']
        
        #calculate system costs
        PitchSystemCost2015 = pitch_system_mass_cost_coeff * pitch_system_mass
        unknowns['pitch_cost'] = PitchSystemCost2015
        
#-------------------------------------------------------------------------------
class SpinnerCost2015(Component):

    def __init__(self):

        super(SpinnerCost2015, self).__init__()

        # variables
        self.add_param('spinner_mass', 0.0, desc='component mass [kg]')
        self.add_param('spinner_mass_cost_coeff', 23.00, desc='spinner/nose cone mass-cost coeff [$/kg]') #mass-cost coeff with default from ppt
    
        # Outputs
        self.add_output('spinner_cost', 0.0, desc='Overall wind turbine component capial costs excluding transportation costs')

    def solve_nonlinear(self, params, unknowns, resids):

        spinner_mass_cost_coeff = params['spinner_mass_cost_coeff']
        spinner_mass = params['spinner_mass']

        #calculate system costs
        SpinnerCost2015 = spinner_mass_cost_coeff * spinner_mass
        unknowns['spinner_cost'] = SpinnerCost2015

#-------------------------------------------------------------------------------
class HubSystemCostAdder2015(Component):

    def __init__(self):

        super(HubSystemCostAdder2015, self).__init__()

        # variables
        self.add_param('hub_cost', 0.0, desc='hub component cost')
        self.add_param('pitch_system_cost', 0.0, desc='pitch system cost')
        self.add_param('spinner_cost', 0.0, desc='spinner component cost')
        
        # multipliers
        self.add_param('hub_assemblyCostMultiplier', 0.0, desc='rotor assembly cost multiplier')
        self.add_param('hub_overheadCostMultiplier', 0.0, desc='rotor overhead cost multiplier')
        self.add_param('hub_profitMultiplier', 0.0, desc='rotor profit multiplier')
        self.add_param('hub_transportMultiplier', 0.0, desc='rotor transport multiplier')
    
        # Outputs
        self.add_output('hub_system_cost', 0.0, desc='Overall wind sub-assembly capial costs including transportation costs')

    def solve_nonlinear(self, params, unknowns, resids):

        hub_cost = params['hub_cost']
        pitch_system_cost = params['pitch_system_cost']
        spinner_cost = params['spinner_cost']
        
        hub_assemblyCostMultiplier = params['hub_assemblyCostMultiplier']
        hub_overheadCostMultiplier = params['hub_overheadCostMultiplier']
        hub_profitMultiplier = params['hub_profitMultiplier']
        hub_transportMultiplier = params['hub_transportMultiplier']

        partsCost = hub_cost + pitch_system_cost + spinner_cost
        
        # updated calculations below to account for assembly, transport, overhead and profit
        unknowns['hub_system_cost'] = (1 + hub_transportMultiplier + hub_profitMultiplier) * ((1 + hub_overheadCostMultiplier + hub_assemblyCostMultiplier) * partsCost)

#-------------------------------------------------------------------------------
class RotorCostAdder2015(Component):
    """
    RotorCostAdder adds up individual rotor system and component costs to get overall rotor cost.
    """

    def __init__(self):
        
        super(RotorCostAdder2015, self).__init__()

        # variables
        self.add_param('blade_cost', 0.0, desc='individual blade cost')
        self.add_param('hub_system_cost', 0.0, desc='cost for hub system')
        
        # parameters
        self.add_param('blade_number', 3, desc='number of rotor blades')
    
        # Outputs
        self.add_output('rotor_cost', 0.0, desc='Overall wind sub-assembly capial costs including transportation costs')

    def solve_nonlinear(self, params, unknowns, resids):

        blade_cost = params['blade_cost']
        blade_number = params['blade_number']
        hub_system_cost = params['hub_system_cost']

        unknowns['rotor_cost'] = blade_cost * blade_number + hub_system_cost

#-------------------------------------------------------------------------------


###### Nacelle
# -------------------------------------------------
class LowSpeedShaftCost2015(Component):

    def __init__(self):

        super(LowSpeedShaftCost2015, self).__init__()

        # variables
        self.add_param('low_speed_shaft_mass', 0.0, desc='component mass [kg]') #mass input
        self.add_param('lss_mass_cost_coeff', 12.60, desc='low speed shaft mass-cost coeff [$/kg]') #mass-cost coeff with default from ppt
    
        # Outputs
        self.add_output('lss_cost', 0.0, desc='Overall wind turbine component capial costs excluding transportation costs') #initialize cost output

    def solve_nonlinear(self, params, unknowns, resids):

        lss_mass_cost_coeff = params['lss_mass_cost_coeff']
        low_speed_shaft_mass = params['low_speed_shaft_mass']

        # calculate component cost
        unknowns['lss_cost'] = lss_mass_cost_coeff * low_speed_shaft_mass

#-------------------------------------------------------------------------------
class BearingsCost2015(Component):

    def __init__(self):

        super(BearingsCost2015, self).__init__()

        # variables
        self.add_param('main_bearing_mass', 0.0, desc='component mass [kg]') #mass input
        self.add_param('bearing_number', 2, desc='number of main bearings []') #number of main bearings- defaults to 2
        self.add_param('bearings_mass_cost_coeff', 6.35, desc='main bearings mass-cost coeff [$/kg]') #mass-cost coeff- HALF of the 12.70 in powerpoint because it was based on TWO bearings
    
        # Outputs
        self.add_output('bearings_cost', 0.0, desc='Overall wind turbine component capial costs excluding transportation costs')

    def solve_nonlinear(self, params, unknowns, resids):

        main_bearing_mass = params['main_bearing_mass']
        bearing_number = params['bearing_number']
        bearings_mass_cost_coeff = params['bearings_mass_cost_coeff']

        #calculate component cost 
        unknowns['bearings_cost'] = bearings_mass_cost_coeff * main_bearing_mass * bearing_number

#-------------------------------------------------------------------------------
class GearboxCost2015(Component):

    def __init__(self):

        super(GearboxCost2015, self).__init__()

        # variables
        self.add_param('gearbox_mass', 0.0, desc='component mass')
        self.add_param('gearbox_mass_cost_coeff', 17.40, desc='gearbox mass-cost coeff [$/kg]') #mass-cost coeff with default from ppt
    
        # Outputs
        self.add_output('gearbox_cost', 0.0, desc='Overall wind turbine component capial costs excluding transportation costs')

    def solve_nonlinear(self, params, unknowns, resids):

        gearbox_mass = params['gearbox_mass']
        gearbox_mass_cost_coeff = params['gearbox_mass_cost_coeff']

        unknowns['gearbox_cost'] = gearbox_mass_cost_coeff * gearbox_mass

#-------------------------------------------------------------------------------
class HighSpeedSideCost2015(Component):

    def __init__(self):
        
        super(HighSpeedSideCost2015, self).__init__()

        # variables
        self.add_param('high_speed_side_mass', 0.0, desc='component mass [kg]')
        self.add_param('high_speed_side_mass_cost_coeff', 8.25, desc='high speed side mass-cost coeff [$/kg]') #mass-cost coeff with default from list
    
        # Outputs
        self.add_output('hss_cost', 0.0, desc='Overall wind turbine component capial costs excluding transportation costs')

    def solve_nonlinear(self, params, unknowns, resids):

        high_speed_side_mass = params['high_speed_side_mass']
        high_speed_side_mass_cost_coeff = params['high_speed_side_mass_cost_coeff']
        
        unknowns['hss_cost'] = high_speed_side_mass_cost_coeff * high_speed_side_mass

#-------------------------------------------------------------------------------
class GeneratorCost2015(Component):

    def __init__(self):

        super(GeneratorCost2015, self).__init__()

        # variables
        self.add_param('generator_mass', 0.0, desc='component mass [kg]')
        self.add_param('generator_mass_cost_coeff', 17.43, desc='generator mass cost coeff [$/kg]') #mass-cost coeff with default from ppt
    
        # Outputs
        self.add_output('generator_cost', 0.0, desc='Overall wind turbine component capial costs excluding transportation costs')

    def solve_nonlinear(self, params, unknowns, resids):

        generator_mass = params['generator_mass']
        generator_mass_cost_coeff = params['generator_mass_cost_coeff']
        
        unknowns['generator_cost'] = generator_mass_cost_coeff * generator_mass

#-------------------------------------------------------------------------------
class BedplateCost2015(Component):

    def __init__(self):
        
        super(BedplateCost2015, self).__init__()
        
        # variables
        self.add_param('bedplate_mass', 0.0, desc='component mass [kg]')
        self.add_param('bedplate_mass_cost_coeff', 4.50, desc='bedplate mass-cost coeff [$/kg]') #mass-cost coeff with default from ppt
    
        # Outputs
        self.add_output('bedplate_cost', 0.0, desc='Overall wind turbine component capial costs excluding transportation costs')

    def solve_nonlinear(self, params, unknowns, resids):

        bedplate_mass = params['bedplate_mass']
        bedplate_mass_cost_coeff = params['bedplate_mass_cost_coeff']

        unknowns['bedplate_cost'] = bedplate_mass_cost_coeff * bedplate_mass

#---------------------------------------------------------------------------------
class YawSystemCost2015(Component):

    def __init__(self):

        super(YawSystemCost2015, self).__init__()

        # variables
        self.add_param('yaw_system_mass', 0.0, desc='component mass [kg]')
        self.add_param('yaw_system_mass_cost_coeff', 11.01, desc='yaw system mass cost coeff [$/kg]') #mass-cost coeff with default from list
    
        # Outputs
        self.add_output('yaw_cost', 0.0, desc='Overall wind turbine component capial costs excluding transportation costs')

    def solve_nonlinear(self, params, unknowns, resids):

        yaw_system_mass = params['yaw_system_mass']
        yaw_system_mass_cost_coeff = params['yaw_system_mass_cost_coeff']
        
        unknowns['yaw_cost'] = yaw_system_mass_cost_coeff * yaw_system_mass

#---------------------------------------------------------------------------------
class VariableSpeedElecCost2015(Component):

    def __init__(self):

        super(VariableSpeedElecCost2015, self).__init__()

        # variables
        self.add_param('variable_speed_elec_mass', 0.0, desc='component mass [kg]')
        self.add_param('variable_speed_elec_mass_cost_coeff', 26.50, desc='variable speed electronics mass cost coeff [$/kg]') #mass-cost coeff with default from list
    
        # Outputs
        self.add_output('vs_cost', 0.0, desc='Overall wind turbine component capial costs excluding transportation costs')

    def solve_nonlinear(self, params, unknowns, resids):

        variable_speed_elec_mass = params['variable_speed_elec_mass']
        variable_speed_elec_mass_cost_coeff = params['variable_speed_elec_mass_cost_coeff']

        unknowns['vs_cost'] = variable_speed_elec_mass_cost_coeff * variable_speed_elec_mass

#---------------------------------------------------------------------------------
class HydraulicCoolingCost2015(Component):

    def __init__(self):

        super(HydraulicCoolingCost2015, self).__init__()

        # variables
        self.add_param('hydraulic_cooling_mass', 0.0, desc='component mass [kg]')
        self.add_param('hydraulic_cooling_mass_cost_coeff', 163.95, desc='hydraulic and cooling system mass cost coeff [$/kg]') #mass-cost coeff with default from list
    
        # Outputs
        self.add_output('hvac_cost', 0.0, desc='Overall wind turbine component capial costs excluding transportation costs')

    def solve_nonlinear(self, params, unknowns, resids):

        hydraulic_cooling_mass = params['hydraulic_cooling_mass']
        hydraulic_cooling_mass_cost_coeff = params['hydraulic_cooling_mass_cost_coeff']

        # calculate cost
        unknowns['hvac_cost'] = hydraulic_cooling_mass_cost_coeff * hydraulic_cooling_mass

#---------------------------------------------------------------------------------
class NacelleCoverCost2015(Component):

    def __init__(self):

        super(NacelleCoverCost2015, self).__init__()

        # variables
        self.add_param('nacelle_cover_mass', 0.0, desc='component mass [kg]')
        self.add_param('nacelle_cover_mass_cost_coeff', 7.61, desc='nacelle cover mass cost coeff [$/kg]') #mass-cost coeff with default from list
    
        # Outputs
        self.add_output('cover_cost', 0.0, desc='Overall wind turbine component capial costs excluding transportation costs')

    def solve_nonlinear(self, params, unknowns, resids):

        nacelle_cover_mass = params['nacelle_cover_mass']
        nacelle_cover_mass_cost_coeff = params['nacelle_cover_mass_cost_coeff']

        unknowns['cover_cost'] = nacelle_cover_mass_cost_coeff * nacelle_cover_mass

#---------------------------------------------------------------------------------
class ElecConnecCost2015(Component):

    def __init__(self):

        super(ElecConnecCost2015, self).__init__()

        # variables
        self.add_param('machine_rating', 0.0, desc='machine rating')
        self.add_param('elec_connec_cost_esc', 1.5, desc='cost esc from 2002 to 2015 for electrical connections') ####KLD update this
        self.add_param('elec_connec_machine_rating_cost_coeff', 40.0, desc='2002 electrical connections cost coeff per kW') #default from old CSM
    
        # Outputs
        self.add_output('elec_cost', 0.0, desc='Overall wind turbine component capial costs excluding transportation costs')

    def solve_nonlinear(self, params, unknowns, resids):

        machine_rating = params['machine_rating']
        elec_connec_cost_esc = params['elec_connec_cost_esc']
        elec_connec_machine_rating_cost_coeff = params['elec_connec_machine_rating_cost_coeff']

        unknowns['elec_cost'] = elec_connec_machine_rating_cost_coeff * machine_rating * elec_connec_cost_esc #esc will be from 2002 $ to 2015 $


#---------------------------------------------------------------------------------
class ControlsCost2015(Component):

    def __init__(self):

        super(ControlsCost2015, self).__init__()

        # variables
        self.add_param('offshore', False, desc='flag for offshore project')
        self.add_param('controls_cost_base', np.array([35000.0,55900.0]), desc='2002 controls cost for [onshore, offshore]') #defaults from old CSM
        self.add_param('controls_esc', 1.5, desc='cost esc from 2002 to 2015 for controls') ####KLD update this
    
        # Outputs
        self.add_output('controls_cost', 0.0, desc='Overall wind turbine component capial costs excluding transportation costs')

    def solve_nonlinear(self, params, unknowns, resids):

        offshore = params['offshore']
        controls_cost_base = params['controls_cost_base']
        controls_esc = params['controls_esc']

        if (not offshore):
            ControlsCost = controls_cost_base[0] * controls_esc
        else:
            ControlsCost  = controls_cost_base[1] * controls_esc
        unknowns['controls_cost'] = ControlsCost

#---------------------------------------------------------------------------------
class OtherMainframeCost2015(Component):

    def __init__(self):  

        super(OtherMainframeCost2015, self).__init__()

        # variables
        self.add_param('nacelle_platforms_mass', 0.0, desc='component mass [kg]')
        self.add_param('nacelle_platforms_mass_cost_coeff', 8.7, desc='nacelle platforms mass cost coeff [$/kg]') #default from old CSM
        self.add_param('crane', False, desc='flag for presence of onboard crane')
        self.add_param('crane_cost', 12000.0, desc='crane cost if present [$]') #default from old CSM
        self.add_param('bedplate_cost', 0.0, desc='component cost [USD]')
        self.add_param('base_hardware_cost_coeff', 0.7, desc='base hardware cost coeff based on bedplate cost') #default from old CSM
    
        # Outputs
        self.add_output('other_cost', 0.0, desc='Overall wind turbine component capial costs excluding transportation costs')

    def solve_nonlinear(self, params, unknowns, resids):

        nacelle_platforms_mass = params['nacelle_platforms_mass']
        nacelle_platforms_mass_cost_coeff = params['nacelle_platforms_mass_cost_coeff']
        crane = params['crane']
        crane_cost = params['crane_cost']
        bedplate_cost = params['bedplate_cost']
        base_hardware_cost_coeff = params['base_hardware_cost_coeff']

        # nacelle platform cost
        NacellePlatformsCost = nacelle_platforms_mass_cost_coeff * nacelle_platforms_mass

        # crane cost
        if (crane):
            craneCost  = crane_cost
        else:
            craneCost  = 0.0

        # base hardware cost
        BaseHardwareCost = bedplate_cost * base_hardware_cost_coeff
    
        #aggregate all three mainframe costs
        unknowns['other_cost'] = (NacellePlatformsCost + craneCost + BaseHardwareCost)

#-------------------------------------------------------------------------------
class TransformerCost2015(Component):

    def __init__(self):

        super(TransformerCost2015, self).__init__()

        # variables
        self.add_param('transformer_mass', 0.0, desc='component mass [kg]')
        self.add_param('transformer_mass_cost_coeff', 26.5, desc='transformer mass cost coeff [$/kg]') #mass-cost coeff with default from ppt
    
        # Outputs
        self.add_output('transformer_cost', 0.0, desc='Overall wind turbine component capial costs excluding transportation costs')

    def solve_nonlinear(self, params, unknowns, resids):

        transformer_mass = params['transformer_mass']
        transformer_mass_cost_coeff = params['transformer_mass_cost_coeff']
        
        unknowns['transformer_cost'] = transformer_mass_cost_coeff * transformer_mass

#-------------------------------------------------------------------------------
class NacelleSystemCostAdder2015(Component):

    def __init__(self):
        
        super(NacelleSystemCostAdder2015, self).__init__()

        # variables
        self.add_param('lss_cost', 0.0, desc='component cost')
        self.add_param('bearing_cost', 0.0, desc='component cost')
        self.add_param('gearbox_cost', 0.0, desc='component cost')
        self.add_param('hss_cost', 0.0, desc='component cost')
        self.add_param('generator_cost', 0.0, desc='component cost')
        self.add_param('bedplate_cost', 0.0, desc='component cost')
        self.add_param('yaw_cost', 0.0, desc='component cost')
        self.add_param('vs_cost', 0.0, desc='component cost')
        self.add_param('hvac_cost', 0.0, desc='component cost')
        self.add_param('cover_cost', 0.0, desc='component cost')
        self.add_param('elec_cost', 0.0, desc='component cost')
        self.add_param('controls_cost', 0.0, desc='component cost')
        self.add_param('other_cost', 0.0, desc='component cost')
        self.add_param('transformer_cost', 0.0, desc='component cost')
        self.add_param('bearing_number', 2, desc ='number of bearings')
        
        #multipliers
        self.add_param('nacelle_assemblyCostMultiplier', 0.0, desc='nacelle assembly cost multiplier')
        self.add_param('nacelle_overheadCostMultiplier', 0.0, desc='nacelle overhead cost multiplier')
        self.add_param('nacelle_profitMultiplier', 0.0, desc='nacelle profit multiplier')
        self.add_param('nacelle_transportMultiplier', 0.0, desc='nacelle transport multiplier')
    
        # returns
        self.add_output('nacelle_cost', 0.0, desc='component cost')

    def solve_nonlinear(self, params, unknowns, resids):

        lss_cost = params['lss_cost']
        bearing_cost = params['bearing_cost']
        gearbox_cost = params['gearbox_cost']
        hss_cost = params['hss_cost']
        generator_cost = params['generator_cost']
        bedplate_cost = params['bedplate_cost']
        yaw_cost = params['yaw_cost']
        vs_cost = params['vs_cost']
        hvac_cost = params['hvac_cost']
        cover_cost = params['cover_cost']
        elec_cost = params['elec_cost']
        controls_cost = params['controls_cost']
        other_cost = params['other_cost']
        transformer_cost = params['transformer_cost']
        bearing_number = params['bearing_number']

        nacelle_assemblyCostMultiplier = params['nacelle_assemblyCostMultiplier']
        nacelle_overheadCostMultiplier = params['nacelle_overheadCostMultiplier']
        nacelle_profitMultiplier = params['nacelle_profitMultiplier']
        nacelle_transportMultiplier = params['nacelle_transportMultiplier']

        # aggregation of nacelle costs
        partsCost = lss_cost + \
                    bearing_number * bearing_cost + \
                    gearbox_cost + \
                    hss_cost + \
                    generator_cost + \
                    bedplate_cost + \
                    yaw_cost + \
                    vs_cost + \
                    hvac_cost + \
                    cover_cost + \
                    elec_cost + \
                    controls_cost + \
                    other_cost + \
                    transformer_cost

        #apply multipliers for assembly, transport, overhead, and profits
        unknowns['nacelle_cost'] = (1 + nacelle_transportMultiplier + nacelle_profitMultiplier) * ((1 + nacelle_overheadCostMultiplier + nacelle_assemblyCostMultiplier) * partsCost)

###### Tower
#-------------------------------------------------------------------------------
class TowerCost2015(Component):

    def __init__(self):
      
        super(TowerCost2015, self).__init__()

        # variables
        self.add_param('tower_mass', 0.0, desc='tower mass [kg]')
        self.add_param('tower_mass_cost_coeff', 3.20, desc='tower mass-cost coeff [$/kg]') #mass-cost coeff with default from ppt
    
        # Outputs
        self.add_output('tower_parts_cost', 0.0, desc='Overall wind turbine component capial costs excluding transportation costs')

    def solve_nonlinear(self, params, unknowns, resids):

        tower_mass = params['tower_mass']
        tower_mass_cost_coeff = params['tower_mass_cost_coeff']
 
        unknowns['tower_parts_cost'] = tower_mass_cost_coeff * tower_mass

#-------------------------------------------------------------------------------
class TowerCostAdder2015(Component):

    def __init__(self):

        super(TowerCostAdder2015, self).__init__()

        # variables
        self.add_param('tower_parts_cost', 0.0, desc='component cost')
      
        # multipliers
        self.add_param('tower_assemblyCostMultiplier', 0.0, desc='tower assembly cost multiplier')
        self.add_param('tower_overheadCostMultiplier', 0.0, desc='tower overhead cost multiplier')
        self.add_param('tower_profitMultiplier', 0.0, desc='tower profit cost multiplier')
        self.add_param('tower_transportMultiplier', 0.0, desc='tower transport cost multiplier')
        
        # returns
        self.add_output('tower_cost', 0.0, desc='component cost') 

    def solve_nonlinear(self, params, unknowns, resids):

        tower_parts_cost = params['tower_parts_cost']

        tower_assemblyCostMultiplier = params['tower_assemblyCostMultiplier']
        tower_overheadCostMultiplier = params['tower_overheadCostMultiplier']
        tower_profitMultiplier = params['tower_profitMultiplier']
        tower_transportMultiplier = params['tower_transportMultiplier']

        partsCost = tower_parts_cost
        unknowns['tower_cost'] = (1 + tower_transportMultiplier + tower_profitMultiplier) * ((1 + tower_overheadCostMultiplier + tower_assemblyCostMultiplier) * partsCost)

#-------------------------------------------------------------------------------
class TurbineCostAdder2015(Component):

    def __init__(self):

        super(TurbineCostAdder2015, self).__init__()

        # Variables
        self.add_param('rotor_cost', 0.0, desc='rotor cost')
        self.add_param('nacelle_cost', 0.0, desc='nacelle cost')
        self.add_param('tower_cost', 0.0, desc='tower cost')
    
        # parameters
        self.add_param('turbine_assemblyCostMultiplier', 0.0, desc='turbine multiplier for assembly cost in manufacturing')
        self.add_param('turbine_overheadCostMultiplier', 0.0, desc='turbine multiplier for overhead')
        self.add_param('turbine_profitMultiplier', 0.0, desc='turbine multiplier for profit markup')
        self.add_param('turbine_transportMultiplier', 0.0, desc='turbine multiplier for transport costs')
    
        # Outputs
        self.add_output('turbine_cost', 0.0, desc='Overall wind turbine capial costs including transportation costs')

    def solve_nonlinear(self, params, unknowns, resids):

        rotor_cost = params['rotor_cost']
        nacelle_cost = params['nacelle_cost']
        tower_cost = params['tower_cost']
        
        turbine_assemblyCostMultiplier = params['turbine_assemblyCostMultiplier']
        turbine_overheadCostMultiplier = params['turbine_overheadCostMultiplier']
        turbine_profitMultiplier = params['turbine_profitMultiplier']
        turbine_transportMultiplier = params['turbine_transportMultiplier']

        partsCost = rotor_cost + nacelle_cost + tower_cost

        unknowns['turbine_cost'] = (1 + turbine_transportMultiplier + turbine_profitMultiplier) * ((1 + turbine_overheadCostMultiplier + turbine_assemblyCostMultiplier) * partsCost)



#-------------------------------------------------------------------------------
class Turbine_CostsSE_2015(Group):

    def __init__(self):
      
        super(Turbine_CostsSE_2015, self).__init__()

        self.add('blade_c',BladeCost2015(), promotes=['*'])
        self.add('hub_c',HubCost2015(), promotes=['*'])
        self.add('pitch_c',PitchSystemCost2015(), promotes=['*'])
        self.add('spinner_c',SpinnerCost2015(), promotes=['*'])
        self.add('hub_adder',HubSystemCostAdder2015(), promotes=['*'])
        self.add('rotor_adder',RotorCostAdder2015(), promotes=['*'])
        self.add('lss_c',LowSpeedShaftCost2015(), promotes=['*'])
        self.add('bearing_c',BearingsCost2015(), promotes=['*'])
        self.add('gearbox_c',GearboxCost2015(), promotes=['*'])
        self.add('hss_c',HighSpeedSideCost2015(), promotes=['*'])
        self.add('generator_c',GeneratorCost2015(), promotes=['*'])
        self.add('bedplate_c',BedplateCost2015(), promotes=['*'])
        self.add('yaw_c',YawSystemCost2015(), promotes=['*'])
        self.add('hvac_c',HydraulicCoolingCost2015(), promotes=['*'])
        self.add('controls_c',ControlsCost2015(), promotes=['*'])
        self.add('vs_c', VariableSpeedElecCost2015(), promotes=['*'])
        self.add('elec_c', ElecConnecCost2015(), promotes=['*'])
        self.add('cover_c',NacelleCoverCost2015(), promotes=['*'])
        self.add('other_c',OtherMainframeCost2015(), promotes=['*'])
        self.add('transformer_c',TransformerCost2015(), promotes=['*'])
        self.add('nacelle_adder',NacelleSystemCostAdder2015(), promotes=['*'])
        self.add('tower_c',TowerCost2015(), promotes=['*'])
        self.add('tower_adder',TowerCostAdder2015(), promotes=['*'])
        self.add('turbine_c',TurbineCostAdder2015(), promotes=['*'])
  

#-------------------------------------------------------------------------------
def example():

    # simple test of module
    turbine = Turbine_CostsSE_2015()
    prob = Problem(turbine)
    prob.setup()

    prob['blade_mass'] = 17650.67  # inline with the windpact estimates
    prob['hub_mass'] = 31644.5
    prob['pitch_system_mass'] = 17004.0
    prob['spinner_mass'] = 1810.5
    prob['low_speed_shaft_mass'] = 31257.3
    #bearingsMass'] = 9731.41
    prob['main_bearing_mass'] = 9731.41 / 2
    prob['gearbox_mass'] = 30237.60
    prob['high_speed_side_mass'] = 1492.45
    prob['generator_mass'] = 16699.85
    prob['bedplate_mass'] = 93090.6
    prob['yaw_system_mass'] = 11878.24
    prob['tower_mass'] = 434559.0
    prob['variable_speed_elec_mass'] = 1000. #Float(iotype='in', units='kg', desc='component mass [kg]')
    prob['hydraulic_cooling_mass'] = 1000. #Float(iotype='in', units='kg', desc='component mass [kg]')
    prob['nacelle_cover_mass'] = 1000. #Float(iotype='in', units='kg', desc='component mass [kg]')
    prob['nacelle_platforms_mass'] = 1000. #Float(iotype='in', units='kg', desc='component mass [kg]')
    prob['transformer_mass'] = 1000. #Float(iotype='in', units='kg', desc='component mass [kg]')    

    # other inputs
    prob['machine_rating'] = 5000.0
    prob['blade_number'] = 3
    prob['crane'] = True
    prob['offshore'] = True
    prob['bearing_number'] = 2

    prob.run()
   
    print "The results for the NREL 5 MW Reference Turbine in an offshore 20 m water depth location are:"
    for io in turbine.unknowns:
        print io + ' ' + str(turbine.unknowns[io])


if __name__ == "__main__":

    example()
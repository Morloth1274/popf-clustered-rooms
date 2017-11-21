/************************************************************************
 * Copyright 2008, Strathclyde Planning Group,
 * Department of Computer and Information Sciences,
 * University of Strathclyde, Glasgow, UK
 * http://planning.cis.strath.ac.uk/
 *
 * Maria Fox, Richard Howey and Derek Long - VAL
 * Stephen Cresswell - PDDL Parser
 *
 * This file is part of VAL, the PDDL validator.
 *
 * VAL is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * VAL is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with VAL.  If not, see <http://www.gnu.org/licenses/>.
 *
 ************************************************************************/

#include <geometry_msgs/PoseStamped.h>
#include <squirrel_navigation_msgs/ClutterPlannerSrv.h>
#include "SimpleEval.h"
#include "TypedAnalyser.h"
#include "instantiation.h"
#include "typecheck.h"
#include "parsing/ptree.h"
#include "ToFunction.h"
#include <vector>

using namespace VAL;

namespace Inst {

bool SimpleEvaluator::verbose = false;

IState InitialStateEvaluator::initState;
IState0Arity InitialStateEvaluator::init0State;
nav_msgs::OccupancyGrid InitialStateEvaluator::grid;
bool InitialStateEvaluator::grid_initialised = false;

//ros::NodeHandle InitialStateEvaluator::nh("popf");
//mongodb_store::MessageStoreProxy InitialStateEvaluator::messageStore(nh);
//bool InitialStateEvaluator::ros_initiated = false;

void InitialStateEvaluator::receiveMap(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
	std::cout << "InitialStateEvaluator::receiveMap" << std::endl;
	grid = *msg;
	grid_initialised = true;
}

bool InitialStateEvaluator::worldToMap(double wx, double wy, unsigned int& mx, unsigned int& my) {
	if (wx < grid.info.origin.position.x || wy < grid.info.origin.position.y)
		return false;

	mx = (int) ((wx - grid.info.origin.position.x) / grid.info.resolution);
	my = (int) ((wy - grid.info.origin.position.y) / grid.info.resolution);

	if (mx < grid.info.width && my < grid.info.height)
		return true;

	return false;
}

void InitialStateEvaluator::fromRadiusToCellVector(const double& radius, squirrel_navigation_msgs::ObjectMSG& obj) {
	int cell_radius = (ceil)(radius / grid.info.resolution);
	for (int i = -cell_radius; i < cell_radius; ++i) {
		for (int j = -cell_radius; j < cell_radius; ++j) {
			if (i * i + j * j <= cell_radius * cell_radius) {
				squirrel_navigation_msgs::CellMSG tmpCell;
				tmpCell.mx = obj.center_cell.mx + i;
				tmpCell.my = obj.center_cell.my + j;
				obj.cell_vector.push_back(tmpCell);
			}
		}
	}
}

void InitialStateEvaluator::transform(const squirrel_manipulation_msgs::GetObjectPositions& object_positions, std::vector<squirrel_navigation_msgs::ObjectMSG>& objects)
{
	objects.clear();
	squirrel_navigation_msgs::ObjectMSG tmp;
	for (unsigned int i = 0; i < object_positions.response.objectids.size(); ++i) {
			tmp.uid = i;
			tmp.center_wx = object_positions.response.objectposes[i].position.x;
			tmp.center_wy = object_positions.response.objectposes[i].position.y;
			unsigned int my, mx;
			if (worldToMap(tmp.center_wx, tmp.center_wy, mx, my)) {
					tmp.center_cell.mx = mx;
					tmp.center_cell.my = my;
					fromRadiusToCellVector(object_positions.response.diameters[i], tmp);
			} else {
					ROS_ERROR("SHOULD NOT HAPPEN IN GET OBJECT CLIENT WORLD TO MAP FAILED");
			}
			objects.push_back(tmp);
	}
	ROS_INFO("SIZE OF OBJECTS IN OBJECT_CLIENT %zu", objects.size());
}

// Bram: Change this function.
void InitialStateEvaluator::setInitialState()
{
	ros::NodeHandle nh("popf");
	ros::Subscriber sub = nh.subscribe("/map", 1, &InitialStateEvaluator::receiveMap);
	
	std::cout << "InitialStateEvaluator::setInitialState() - waiting for the static map te become available." << std::endl;
	while (!grid_initialised)
	{
		ros::spinOnce();
	}
	
	std::cout << "InitialStateEvaluator::setInitialState() - static map is now available!" << std::endl;
	
	mongodb_store::MessageStoreProxy messageStore(nh);
	ros::ServiceClient client = nh.serviceClient<squirrel_navigation_msgs::ClutterPlannerSrv>("clutter_service");
	ros::ServiceClient find_objects_service = nh.serviceClient<squirrel_manipulation_msgs::GetObjectPositions>("/getObjectsPositions");
	
	initState.clear();
	init0State.clear();
	
	std::vector<std::pair<parameter_symbol, parameter_symbol> > waypoints;
	std::map<std::string, geometry_msgs::PoseStamped> retreived_waypoints;
	
	//mongodb_store::MessageStoreProxy messageStore(*nh);
	for (const_symbol_list::const_iterator ci = current_analysis->the_problem->objects->begin(); ci != current_analysis->the_problem->objects->end(); ++ci)
	{
		VAL::const_symbol* s = *ci;
		if (s->type->getName() == "waypoint")
		{
			for (const_symbol_list::const_iterator ci2 = current_analysis->the_problem->objects->begin(); ci2 != current_analysis->the_problem->objects->end(); ++ci2)
			{
				VAL::const_symbol* s2 = *ci2;
				if (s2->type->getName() == "waypoint")
				{
					// Check if these two waypoints are connected.s
					std::cout << "Are " << s->getName() << " and " << s2->getName() << " connected?" << std::endl;
					
					geometry_msgs::PoseStamped wp1_loc;
					{
						std::vector<boost::shared_ptr<geometry_msgs::PoseStamped> >results;
						if (!messageStore.queryNamed<geometry_msgs::PoseStamped>(s->getName(), results) || results.empty())
						{
							std::cerr << "Could not query the message store for the waypoint: " << s->getName().c_str() << std::endl;
							exit(1);
						}
						wp1_loc = *results[0];
					}
					
					geometry_msgs::PoseStamped wp2_loc;
					{
						std::vector<boost::shared_ptr<geometry_msgs::PoseStamped> >results;
						if (!messageStore.queryNamed<geometry_msgs::PoseStamped>(s2->getName(), results) || results.empty())
						{
							std::cout << "Could not query the message store for the waypoint: " <<  s2->getName().c_str() << std::endl;
							exit(1);
						}
						wp2_loc = *results[0];
					}
					
					// Now we can call the pathplanner to initialise the initial state.
					std::cout << s->getName() << " = (" << wp1_loc.pose.position.x << ", " << wp1_loc.pose.position.y << ", " << wp1_loc.pose.position.z << ")" << std::endl;
					std::cout << s2->getName() << " = (" << wp2_loc.pose.position.x << ", " << wp2_loc.pose.position.y << ", " << wp2_loc.pose.position.z << ")" << std::endl;
					
					std::vector<squirrel_navigation_msgs::ObjectMSG> objects;
					
					// Get all the objects.
					squirrel_manipulation_msgs::GetObjectPositions op;
					if (!find_objects_service.call(op))
					{
						ROS_ERROR("KCL: (SquirrelPlanningCluttered) Could not call the server to get all the objects in the domain!");
						exit(1);
					}
					transform(op, objects);
					
					squirrel_navigation_msgs::ClutterPlannerSrv srv;
					srv.request.goal = wp2_loc;
					srv.request.start = wp1_loc;
					srv.request.obstacles_in = objects;
					srv.request.grid = grid;
					
					if (!client.call(srv))
					{
						std::cout << "Not connected: " << s->getName() << " -> " << s2->getName() << std::endl;
						continue;
					} else {
						std::cout << "Connected: " << s->getName() << " -> " << s2->getName() << std::endl;
					}
					
					// Create a literal for this.
					pred_symbol * p = current_analysis->pred_tab.symbol_get("connected");
					std::cout << "Found predicate symbol at: " << p << std::endl;
					
					holding_pred_symbol* hps = dynamic_cast<holding_pred_symbol*>(p);
					
					std::cout << " ===== Exising extended pred symbol ===== Address: " << hps << std::endl;
					
					for (holding_pred_symbol::PIt pit = hps->pBegin(); pit != hps->pEnd(); ++pit)
					{
						extended_pred_symbol* eps = *pit;
						std::cout << "\tProcess: " << eps->getName() << " (" << eps << ")" << std::endl;
					}
					
					// Lookup the extended predicate symbol with the correct types.
					std::vector<VAL::pddl_type*> pddl_types;
					pddl_types.push_back(s->type);
					pddl_types.push_back(s2->type);
					extended_pred_symbol* eps = hps->find<std::vector<VAL::pddl_type*>::const_iterator>(p, pddl_types.begin(), pddl_types.end());
					
					std::cout << "\tFound the pred symbol: " << eps->getName() << " (" << eps << ")" << std::endl;
					
					std::cout << "Initialise the initial state for: " << p->getName() << std::endl;
					parameter_symbol_list* var_list = new parameter_symbol_list();
					var_list->push_back(s);
					var_list->push_back(s2);
					
					proposition* prop = new proposition(eps, var_list);
					eps->setInitial(prop);
					
					simple_effect* se = new simple_effect(prop);
					current_analysis->the_problem->initial_state->add_effects.push_back(se);

					std::cout << "Insert " << prop->head->getName() << " (" << prop->head << ") Into the initial state!" << std::endl;
				}
			}
		}
	}
	
	for(pc_list<simple_effect*>::const_iterator i = 
				current_analysis->the_problem->initial_state->add_effects.begin();
				i != current_analysis->the_problem->initial_state->add_effects.end();++i)
	{
		if((*i)->prop->args->begin()==(*i)->prop->args->end())
		{
			// Arity 0...
			init0State.insert((*i)->prop->head);
		}
		else
		{
			std::cout << "Insert " << (*i)->prop->head->getName() << " (" << (*i)->prop->head << ") Into the initial state!" << std::endl;
			initState[(*i)->prop->head].push_back((*i)->prop->args);
		};
	};
	
	std::cout << "Post state..." << std::endl;
	for (IState::const_iterator ci = initState.begin(); ci != initState.end(); ++ci)
	{
		const vector<VAL::parameter_symbol_list*>& vpsl = ci->second;
		for (std::vector<VAL::parameter_symbol_list*>::const_iterator ci2 = vpsl.begin(); ci2 != vpsl.end(); ++ci2)
		{
			std::cout << "\t(" << ci->first->getName();
			const VAL::parameter_symbol_list* psl = *ci2;
			for (VAL::parameter_symbol_list::const_iterator ci3 = psl->begin(); ci3 != psl->end(); ++ci3)
			{
				std::cout << " ";// << (*ci3)->getName();
				
				(*ci3)->write(std::cout);
			}
			std::cout << ")" << std::endl;
		}
	}
};

void InitialStateEvaluator::evaluateSimpleGoal(FastEnvironment * f,simple_goal * s)
{
	extended_pred_symbol * eps = EPS(s->getProp()->head);
	if(eps->appearsStatic())
	{
		if(!eps->isCompletelyStatic(f,s->getProp()))
		{
//			cout << s->getProp()->head->getName() << " is a faker\n";
			unknownTrue = true;
			unknownFalse = true;
			return;
		};
		
		if (SimpleEvaluator::verbose) cout << s->getProp()->head->getName() << " is static\n";

		unknownTrue = false;
		unknownFalse = false;

		//eps = eps->getPrimitive(f,s->getProp());

		if(eps->contains(f,s->getProp()))
		{
			valueTrue = true;
			valueFalse = false;
		}
		else
		{
			valueTrue = (init0State.find(s->getProp()->head) != init0State.end());
			valueFalse = !valueTrue;
		}
		if(s->getPolarity() == E_NEG)
		{
			const bool vt = valueTrue;
			valueTrue = valueFalse;
			valueFalse = vt;
		};

		return;
	}
	else if(eps->cannotIncrease())
	{
//		cout << "Got one that cannot increase " << *eps << "\n";
		if(s->getPolarity() == E_NEG)
		{
			valueTrue = !valueTrue;
			valueFalse = !valueFalse;
			unknownTrue = true;
			unknownFalse = false;
			return;
		}
		unknownTrue = false;
		unknownFalse = false;
		if(eps->contains(f,s->getProp()))
		{
			valueTrue = true;
			valueFalse = false;
			return;
		}; 
		valueTrue = (init0State.find(s->getProp()->head) != init0State.end());
		valueFalse = !valueTrue;
		return;
		
	}
	unknownTrue = true;
	unknownFalse = true;
};


bool partialMatch(const VAL::const_symbol * x,const VAL::const_symbol * y)
{
	return x==y || x==0 || y==0;
};

void SimpleEvaluator::visit_preference(preference * p)
{};

bool SimpleEvaluator::equiv(const parameter_symbol_list * s,const parameter_symbol_list * p)
{
	parameter_symbol_list::const_iterator y = p->begin();
	for(parameter_symbol_list::const_iterator x = s->begin();x != s->end();++x,++y)
	{
		if((*f)[*x] != *y) return false;
	};
	return true;
};
					
void SimpleEvaluator::visit_simple_goal(simple_goal * s)
{
	if(EPS(s->getProp()->head)->getParent() == this->equality)
	{
//	cout << "Got equality\n";
		unknownTrue = false;
		unknownFalse = false;
		valueTrue = ((*f)[s->getProp()->args->front()] == 
						(*f)[s->getProp()->args->back()]);
		valueFalse = !valueTrue;
		
		if(s->getPolarity() == E_NEG)
		{
			const bool vt = valueTrue;
			valueTrue = valueFalse;
			valueFalse = vt;
		};
		return;
	};
	primev->evaluateSimpleGoal(f,s);
	if (SimpleEvaluator::verbose) {
        Literal toPrint(s->getProp(),f);
		if (!unknownTrue && valueTrue) {
			cout << "\t\tValue of fact " << toPrint << "known to be true\n";
		}
		if (!unknownFalse && valueFalse) {
            cout << "\t\tValue of fact " << toPrint << "known to be false\n";
		}
		if (unknownTrue || unknownFalse) {
            cout << "\t\tValue of fact " << toPrint << "unknown\n";
		}
	}
};


void SimpleEvaluator::visit_qfied_goal(qfied_goal * p)
{

	vector<vector<VAL::const_symbol*>::const_iterator> vals(p->getVars()->size());
	vector<vector<VAL::const_symbol*>::const_iterator> starts(p->getVars()->size());
	vector<vector<VAL::const_symbol*>::const_iterator> ends(p->getVars()->size());
	vector<VAL::var_symbol *> vars(p->getVars()->size());
	FastEnvironment fe(*f);
	fe.extend(vars.size());
	int i = 0;
	int c = 1;
	for(var_symbol_list::const_iterator pi = p->getVars()->begin();
			pi != p->getVars()->end();++pi,++i)
	{
		if(instantiatedOp::getValues().find((*pi)->type) == instantiatedOp::getValues().end()) 
		{
			instantiatedOp::getValues()[(*pi)->type] = tc->range(*pi);
		};
		vals[i] = starts[i] = instantiatedOp::getValues()[(*pi)->type].begin();
		ends[i] = instantiatedOp::getValues()[(*pi)->type].end();
		if(ends[i]==starts[i]) return;
		fe[(*pi)] = *(vals[i]);
		vars[i] = *pi;
		c *= instantiatedOp::getValues()[(*pi)->type].size();
	};

	
	valueTrue = (p->getQuantifier() == VAL::E_FORALL);
	valueFalse = !valueTrue;
	unknownTrue = false;
	unknownFalse = false;

	bool uTrue = false;
	bool uFalse = false;

	--i;
	while(vals[i] != ends[i])
	{
// This is inefficient because it creates a copy of the environment even if the copy is never used.
// In practice, this should not be a problem because a quantified effect presumably uses the variables
// it quantifies.
		FastEnvironment * const ecpy = f;
		FastEnvironment toPass(fe);
		f = &toPass;
		p->getGoal()->visit(this);
		
		if (p->getQuantifier() == VAL::E_FORALL) {
;			if(reallyFalse()) {
				if (SimpleEvaluator::verbose) cout << "Contradictory child of forall\n";
				return;
			}
			uTrue = uTrue || unknownTrue;
			uFalse = uFalse || unknownFalse;
		} else {
			if(reallyTrue()) {
				if (SimpleEvaluator::verbose) cout << "Tautologous child of exists\n";
				return;
			}
			uTrue = uTrue || unknownTrue;
			uFalse = uFalse || unknownFalse;
		}
		f = ecpy;

		int x = 0;
		++vals[0];
		if(vals[0] != ends[0]) fe[vars[0]] = *(vals[0]);
		while(x < i && vals[x] == ends[x])
		{
			vals[x] = starts[x];
			fe[vars[x]] = *(vals[x]);
			++x;
			++vals[x];
			if(vals[x] != ends[x]) fe[vars[x]] = *(vals[x]);
		};
	};
	unknownTrue = uTrue;
	unknownFalse = uFalse;
};

void SimpleEvaluator::visit_conj_goal(conj_goal * c)
{
	if (SimpleEvaluator::verbose) cout << "And...\n";
	bool uTrue = false;
	bool uFalse = false;

	unknownTrue = false;
	unknownFalse = false;
	valueTrue = true;
	valueFalse = false;
	for(goal_list::const_iterator i = c->getGoals()->begin();
		i != c->getGoals()->end();++i)
	{
		(*i)->visit(this);
		if(reallyFalse()) {
			if (SimpleEvaluator::verbose) cout << "Contradictory child of and\n";
			return;
		}
		uTrue = uTrue || unknownTrue;
		uFalse = uFalse || unknownFalse;
	};
	unknownTrue = uTrue;
	unknownFalse = uFalse;
        
        if (SimpleEvaluator::verbose) {
            if (!unknownTrue && valueTrue) {
                cout << "\t\tValue of AND known to be true\n";
            }
            if (!unknownFalse && valueFalse) {
                cout << "\t\tValue of AND known to be false\n";
            }
            if (unknownTrue) {                
                cout << "\t\tValue of AND might be true\n";
            }
            if (unknownFalse) {                
                cout << "\t\tValue of AND might be false\n";
            }

        }
};
	
void SimpleEvaluator::visit_disj_goal(disj_goal * d)
{
	if (SimpleEvaluator::verbose) cout << "Or...\n";
	bool uTrue = false;
	bool uFalse = false;

	unknownTrue = false;
	unknownFalse = false;
	valueTrue = false;
	valueFalse = true;

	for(goal_list::const_iterator i = d->getGoals()->begin();
		i != d->getGoals()->end();++i)
	{
		(*i)->visit(this);
		if(reallyTrue()) {
			if (SimpleEvaluator::verbose) cout << "Tautologous child of or\n";
			return;
		}
		uTrue = uTrue || unknownTrue;
		uFalse = uFalse || unknownFalse;
	};
	unknownTrue = uTrue;
	unknownFalse = uFalse;
};

void SimpleEvaluator::visit_timed_goal(timed_goal * t)
{
	t->getGoal()->visit(this);
};

void SimpleEvaluator::visit_imply_goal(imply_goal * ig)
{
	if (SimpleEvaluator::verbose) cout << "Implies...\n";
	ig->getAntecedent()->visit(this);
	if(unknownTrue || unknownFalse) {
		if (SimpleEvaluator::verbose) cout << "Implication with an unknown antecedent\n";
		unknownTrue = true;
		unknownFalse = true;
		return;
	}
	if(valueTrue)
	{
		if (SimpleEvaluator::verbose) cout << "Antecedent tautologous, checking consequent\n";
		ig->getConsequent()->visit(this);
	}
	else
	{
		if (SimpleEvaluator::verbose) cout << "Antecedent contradictory, ex falso quodlibet\n";
		valueTrue = true;
		valueFalse = false;
	}
};

void SimpleEvaluator::visit_neg_goal(neg_goal * ng)
{
	if (SimpleEvaluator::verbose) cout << "Negating...\n";
	ng->getGoal()->visit(this);
	if(!unknownTrue && !unknownFalse)
	{
		const bool vt = valueTrue;
		valueTrue = valueFalse;
		valueFalse = vt;
	} else {
		unknownTrue = true;
		unknownFalse = true;
	}

	if (SimpleEvaluator::verbose) {
		if (valueTrue) {
			cout << "Now cast as true\n";
		} else if (valueFalse) {
			cout << "Now cast as false\n";
		}
	}
};

void SimpleEvaluator::visit_event(event * op)
{
	op->precondition->visit(this);
};

void SimpleEvaluator::visit_process(process * op)
{
	op->precondition->visit(this);
};


void SimpleEvaluator::visit_comparison(comparison * c)
{
//	unknown = true;
//	return;
	
	isFixed = false;
	undefined = false;
	isDuration = false;
	c->getLHS()->visit(this);
	if(undefined) 
	{
		unknownTrue = false;
		valueTrue = false;
		unknownFalse = false;
		valueFalse = false;

		return;
	};
	if(isDuration)
	{
		valueTrue = true;
		unknownTrue = false;
		valueFalse = false;
		unknownFalse = false;
		return;
	};
	bool lhsFixed = isFixed;
	double lhsval = nvalue;
	//bool lhsDur = isDuration;
	
	isDuration = false;
	c->getRHS()->visit(this);
	if(undefined)
	{
		unknownTrue = valueTrue = false;
		unknownFalse = valueFalse = false;
		return;
	};
	
	isFixed &= lhsFixed;
	if(isFixed)
	{
		unknownTrue = false;
		unknownFalse = false;
		switch(c->getOp())
		{
			case E_GREATER:
				valueTrue = (lhsval > nvalue);  // I think this is a problem case if 
											// we are comparing with ?duration in the
											// special duration field.... 
				break;
			case E_GREATEQ:
				valueTrue = (lhsval >= nvalue);
				break;
			case E_LESS:
				valueTrue = (lhsval < nvalue);
				break;
			case E_LESSEQ:
				valueTrue = (lhsval <= nvalue);
				break;
			default: // E_EQUALS
				valueTrue = (lhsval == nvalue);
		};
		valueFalse = !valueTrue;
	}
	else
	{
		unknownTrue = true;
		unknownFalse = true;
	};
};

void SimpleEvaluator::visit_action(action * op)
{
	if (op->precondition) {
            if (SimpleEvaluator::verbose) cout << "Visiting operator preconditions\n";
            op->precondition->visit(this);
            if (SimpleEvaluator::verbose) {
                if(reallyTrue()) {
                    cout << "Preconditions are really true\n";
                }
                if (reallyFalse()) {
                    cout << "Preconditions are really false\n";
                }                
            }
        }
};

void SimpleEvaluator::visit_derivation_rule(derivation_rule * drv)
{
	if (drv->get_body()) drv->get_body()->visit(this);
};

void SimpleEvaluator::visit_durative_action(durative_action * da)
{
	if(da->precondition) da->precondition->visit(this);
	if(unknownTrue || valueTrue)
	{
		da->dur_constraint->visit(this);
	};
};



void SimpleEvaluator::visit_plus_expression(plus_expression * s)
{
	s->getLHS()->visit(this);
	double x = nvalue;
	bool lisFixed = isFixed;
	s->getRHS()->visit(this);
	nvalue += x;
	isFixed &= lisFixed;
};

void SimpleEvaluator::visit_minus_expression(minus_expression * s)
{
	s->getLHS()->visit(this);
	double x = nvalue;
	bool lisFixed = isFixed;
	s->getRHS()->visit(this);
	nvalue -= x;
	isFixed &= lisFixed;
};

void SimpleEvaluator::visit_mul_expression(mul_expression * s)
{
	s->getLHS()->visit(this);
	double x = nvalue;
	bool lisFixed = isFixed;
	s->getRHS()->visit(this);
	nvalue *= x;
	isFixed &= lisFixed;
};

void SimpleEvaluator::visit_div_expression(div_expression * s)
{
	s->getRHS()->visit(this);
	double x = nvalue;
	bool risFixed = isFixed;
	s->getLHS()->visit(this);
	isFixed &= risFixed;
	if(x != 0)
	{
		nvalue /= x;
	};
	if(isFixed && x == 0)
	{
		//cout << "Division by zero!\n";
		undefined = true;
	};
};

void SimpleEvaluator::visit_uminus_expression(uminus_expression * s)
{
	s->getExpr()->visit(this);
};

void SimpleEvaluator::visit_int_expression(int_expression * s)
{
	isFixed = true;
	nvalue = s->double_value();
};

void SimpleEvaluator::visit_float_expression(float_expression * s)
{
	isFixed = true;
	nvalue = s->double_value();
};

void SimpleEvaluator::visit_special_val_expr(special_val_expr * s)
{
	if(s->getKind() == E_DURATION_VAR) isDuration = true;
	isFixed = true; // Possibly inappropriate...
};

void SimpleEvaluator::visit_func_term(func_term * s)
{
	extended_func_symbol * efs = EFT(s->getFunction());
	//cout << "Eval: " << s->getFunction()->getName() << "\n";
	if(efs->isStatic())
	{
		isFixed = true;
		pair<bool,double> pv = efs->getInitial(makeIterator(f,s->getArgs()->begin()),
						makeIterator(f,s->getArgs()->end()));
		if(pv.first)
		{
			nvalue = pv.second;
			//cout << "Value is " << nvalue << "\n";
		}
		else
		{
			undefined = true;
			//cout << "Undefined\n";
		};
	}
	else
	{
		isFixed = false;
		//cout << "Variable\n";
	};
};

};

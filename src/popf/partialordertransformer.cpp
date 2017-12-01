/************************************************************************
 * Copyright 2010, Strathclyde Planning Group,
 * Department of Computer and Information Sciences,
 * University of Strathclyde, Glasgow, UK
 * http://planning.cis.strath.ac.uk/
 *
 * Andrew Coles, Amanda Coles - Code for POPF
 * Maria Fox, Richard Howey and Derek Long - Code from VAL
 * Stephen Cresswell - PDDL Parser
 *
 * This file is part of the planner POPF.
 *
 * POPF is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * POPF is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with POPF.  If not, see <http://www.gnu.org/licenses/>.
 *
 ************************************************************************/

#include <ros/ros.h>
#include <mongodb_store/message_store.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <squirrel_navigation_msgs/ClutterPlannerSrv.h>

#include "partialordertransformer.h"
#include "temporalconstraints.h"
#include "numericanalysis.h"
#include "RPGBuilder.h"

using Inst::Literal;
using std::endl;

static bool applyDebug;

namespace Planner
{

TemporalConstraints * PartialOrderTransformer::cloneTemporalConstraints(const TemporalConstraints * const other, const int extendBy)
{
    return new TemporalConstraints(*other, extendBy);
}

TemporalConstraints * PartialOrderTransformer::emptyTemporalConstraints()
{
    return new TemporalConstraints();
}

// Bram: to change.
void POTHelper_updateForPathfinder(MinimalState & theState, const ActionSegment & a, StepAndBeforeOrAfter stepBA)
{
	const unsigned int stepID = stepBA.stepID;
	const PropositionAnnotation afterNow(stepID);
	
	// If an action was executed that could change the location of the objects we need to recheck the connectivity
	// of all waypoints in the domain.
	std::cout << "****[POTHelper_updateForPathfinder] For action: " << a.first->forOp()->name->getName() << std::endl;
	
	//a.first->writeAllLiterals(std::cout);
	//std::cout << std::endl;
	
	//theState.printState(std::cout);
	//std::cout << std::endl;
	
	// Check which action was called and call the path planner to update the connected properties.
	if (a.first->forOp()->name->getName() == "push_object")
	{
		ros::NodeHandle nh("popf");
		mongodb_store::MessageStoreProxy messageStore(nh);
		ros::ServiceClient client = nh.serviceClient<squirrel_navigation_msgs::ClutterPlannerSrv>("clutter_service");
		ros::ServiceClient find_objects_service = nh.serviceClient<squirrel_manipulation_msgs::GetObjectPositions>("/getObjectsPositions");

		// Get all the objects.
		std::vector<squirrel_navigation_msgs::ObjectMSG> objects;
		squirrel_manipulation_msgs::GetObjectPositions op;
		if (!find_objects_service.call(op))
		{
			ROS_ERROR("KCL: (SquirrelPlanningCluttered) Could not call the server to get all the objects in the domain!");
			exit(1);
		}
		
		// Move the object that is pushed to the correct location.
		VAL::var_symbol_list::const_iterator ci = a.first->forOp()->parameters->begin();
		const VAL::var_symbol* robot = *ci++;
		const VAL::var_symbol* object = *ci++;
		const VAL::var_symbol* from = *ci++;
		const VAL::var_symbol* to = *ci++;
		const VAL::var_symbol* push_location = *ci++;
		const VAL::var_symbol* object_location = *ci;
		
		std::cout << "- Robot: " << robot->getName() << std::endl;
		std::cout << "- Object: " << object->getName() << std::endl;
		std::cout << "- From: " << from->getName() << std::endl;
		std::cout << "- To: " << to->getName() << std::endl;
		std::cout << "- Push Location: " << push_location->getName() << std::endl;
		std::cout << "- Object Location: " << object_location->getName() << std::endl;
		
		InitialStateEvaluator::transform(op, objects);
		
		// Create all literals that are related to 'connected to'.
		for (VAL::const_symbol_list::const_iterator ci = current_analysis->the_problem->objects->begin(); ci != current_analysis->the_problem->objects->end(); ++ci)
		{
			VAL::const_symbol* s = *ci;
			if (s->type->getName() == "waypoint")
			{
				for (VAL::const_symbol_list::const_iterator ci2 = current_analysis->the_problem->objects->begin(); ci2 != current_analysis->the_problem->objects->end(); ++ci2)
				{
					bool previouslyDeleted = false;
					VAL::const_symbol* s2 = *ci2;
					if (s2->type->getName() == "waypoint")
					{
						// Check if these two waypoints are connected.s
						//std::cout << "Are " << s->getName() << " and " << s2->getName() << " connected?" << std::endl;
						
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
						//std::cout << s->getName() << " = (" << wp1_loc.pose.position.x << ", " << wp1_loc.pose.position.y << ", " << wp1_loc.pose.position.z << ")" << std::endl;
						//std::cout << s2->getName() << " = (" << wp2_loc.pose.position.x << ", " << wp2_loc.pose.position.y << ", " << wp2_loc.pose.position.z << ")" << std::endl;
						
						
						squirrel_navigation_msgs::ClutterPlannerSrv srv;
						srv.request.goal = wp2_loc;
						srv.request.start = wp1_loc;
						srv.request.obstacles_in = objects;
						srv.request.grid = InitialStateEvaluator::getGrid();
						
						if (!client.call(srv))
						{
							continue;
							//std::cout << "Connected: " << s->getName() << " -> " << s2->getName() << std::endl;
						} else {
							//std::cout << "Not connected: " << s->getName() << " -> " << s2->getName() << std::endl;
						}
						
						// Create a literal for this.
						pred_symbol * p = current_analysis->pred_tab.symbol_get("connected");
						
						VAL::holding_pred_symbol* hps = dynamic_cast<VAL::holding_pred_symbol*>(p);
						
						// Lookup the extended predicate symbol with the correct types.
						std::vector<VAL::pddl_type*> pddl_types;
						pddl_types.push_back(s->type);
						pddl_types.push_back(s2->type);
						VAL::extended_pred_symbol* eps = hps->find<std::vector<VAL::pddl_type*>::const_iterator>(p, pddl_types.begin(), pddl_types.end());
						
						VAL::parameter_symbol_list* var_list = new VAL::parameter_symbol_list();
						var_list->push_back(s);
						var_list->push_back(s2);
						
						// Probably should not make a new proposition, but look it up from the grounded actions (wherever they are!).
						VAL::proposition* prop = new VAL::proposition(eps, var_list);
						
						// Need to get the literal!
						// Find the literal.
						Literal* l = new Literal(prop, a.first->getEnv());
						//std::cout << "Constructed the literal: ";
						//l->write(std::cout);
						//std::cout << std::endl;
						
						Literal* real_l = instantiatedOp::findLiteral(l);
						
						//std::cout << "Which maps to the literal: ";
						if (real_l == nullptr)
						{
							/*
							std::cout << "Nothing!" << std::endl;
							for (auto ci = instantiatedOp::literalsBegin(); ci != instantiatedOp::literalsEnd(); ++ci)
							{
								(*ci)->write(std::cout);
								std::cout << " (" << *ci << ")" << std::endl;
							}
							*/
							continue;
						}/* else {
							real_l->write(std::cout);
							std::cout << std::endl;
						}*/
						
						int litID = real_l->getID();
						//std::cout << "The ID is: " << litID << std::endl;
						
						const map<int, PropositionAnnotation>::iterator stateItr = theState.retired.find(litID);
						if (stateItr != theState.retired.end()) {
							assert(theState.first.find(litID) == theState.first.end());
							previouslyDeleted = true;
							{
								const StepAndBeforeOrAfter deletedAt = stateItr->second.negativeAvailableFrom;
								assert(!deletedAt.never());

								if (deletedAt.stepID != stepID) {
										theState.temporalConstraints->addOrdering(deletedAt.stepID, stepID, (deletedAt.beforeOrAfter == StepAndBeforeOrAfter::AFTER));
								}
									//cout << "\t" << *real_l << " had previously been deleted " << deletedAt;
							}

							PropositionAnnotation & toUpdate = theState.first.insert(*stateItr).first->second;
							toUpdate.markAsAdded(stepBA);
							theState.retired.erase(stateItr);

							//cout << ", and is now available from " << stepBA << "\n";
						}

						if (!previouslyDeleted) {
							const pair<map<int, PropositionAnnotation>::iterator, bool> stateItrPair = theState.first.insert(make_pair(litID, afterNow));

							if (!stateItrPair.second) { // if we haven't just added it (i.e. it was there already)
								if (stateItrPair.first->second.availableFrom.stepID || stateItrPair.first->second.availableFrom.beforeOrAfter == StepAndBeforeOrAfter::AFTER) {
									//cout << "\t" << *real_l << " used to be available from step " << stateItrPair.first->second.availableFrom.stepID << ", adding ordering constraint\n";
									theState.temporalConstraints->addOrdering(stateItrPair.first->second.availableFrom.stepID, stepID, false);
								}
								stateItrPair.first->second.availableFrom.stepID = stepID; // override when it's available from
								stateItrPair.first->second.availableFrom.beforeOrAfter = StepAndBeforeOrAfter::AFTER; // ...  to after now

							}

							//cout << "\t" << *real_l << " is brand new and available from " << stepBA << "\n";
						}
					}
				}
			}
		}
	}
}

void POTHelper_updateForInstantaneousEffects(MinimalState & theState, const StepAndBeforeOrAfter & stepBA, list<Literal*> & delEffs, list<Literal*> & addEffs)
{
    const unsigned int stepID = stepBA.stepID;
    {
        list<Literal*>::iterator effItr = delEffs.begin();
        const list<Literal*>::iterator effEnd = delEffs.end();

        for (; effItr != effEnd; ++effItr) {
            const int litID = (*effItr)->getID();
            bool matchedFirst = false;
            {
                const map<int, PropositionAnnotation>::iterator stateItr = theState.first.find(litID);
                if (stateItr != theState.first.end()) {

                    assert(theState.retired.find(litID) == theState.retired.end());

                    {
                        const map<StepAndBeforeOrAfter, bool> & invAfter = stateItr->second.deletableFrom;
                        map<StepAndBeforeOrAfter, bool>::const_iterator iamItr = invAfter.begin();
                        const map<StepAndBeforeOrAfter, bool>::const_iterator iamEnd = invAfter.end();
                        for (; iamItr != iamEnd; ++iamItr) {
                            const StepAndBeforeOrAfter * const iaItr = &(iamItr->first);
                            if (iaItr->stepID != stepID) {
                                theState.temporalConstraints->addOrdering(iaItr->stepID, stepID, (iaItr->beforeOrAfter == StepAndBeforeOrAfter::AFTER));
                            }
                        }
                    }
                    {
                        const StepAndBeforeOrAfter addedAt = stateItr->second.availableFrom;
                        if (addedAt.stepID > 0 || addedAt.beforeOrAfter == StepAndBeforeOrAfter::AFTER) {
                            theState.temporalConstraints->addOrdering(addedAt.stepID, stepID, (addedAt.beforeOrAfter == StepAndBeforeOrAfter::AFTER));
                        }
                    }


                    PropositionAnnotation & toUpdate = theState.retired.insert(*stateItr).first->second;
                    toUpdate.markAsDeleted(stepBA);
                    theState.first.erase(stateItr);

                    if (applyDebug) {
                        cout << "\t" << *(*effItr) << " was true, but has now been deleted\n";
                    }

                    matchedFirst = true;
                }
            }

            if (!matchedFirst) {
                const pair<map<int, PropositionAnnotation>::iterator, bool> stateItrPair = theState.retired.insert(make_pair(litID, PropositionAnnotation(false)));
                const map<int, PropositionAnnotation>::iterator & stateItr = stateItrPair.first;
                if (!stateItrPair.second) {

                    {
                        const map<StepAndBeforeOrAfter, bool> & invAfter = stateItr->second.deletableFrom;
                        map<StepAndBeforeOrAfter, bool>::const_iterator iamItr = invAfter.begin();
                        const map<StepAndBeforeOrAfter, bool>::const_iterator iamEnd = invAfter.end();
                        for (; iamItr != iamEnd; ++iamItr) {
                            const StepAndBeforeOrAfter * const iaItr = &(iamItr->first);
                            if (iaItr->stepID != stepID) {
                                theState.temporalConstraints->addOrdering(iaItr->stepID, stepID, (iaItr->beforeOrAfter == StepAndBeforeOrAfter::AFTER));
                            }
                        }
                    }
                    {
                        const StepAndBeforeOrAfter deletedAt = stateItr->second.negativeAvailableFrom;
                        assert(!deletedAt.never());
                        theState.temporalConstraints->addOrdering(deletedAt.stepID, stepID, (deletedAt.beforeOrAfter == StepAndBeforeOrAfter::AFTER));
                    }

                    stateItr->second.markAsDeleted(stepBA);

                    if (applyDebug) {
                        cout << "\t" << *(*effItr) << " was previously deleted, and has been deleted again\n";
                    }
                } else {

                    stateItr->second.markAsDeleted(stepBA);

                    if (applyDebug) {
                        cout << "\t" << *(*effItr) << " was false initially, and has just been deleted\n";
                    }

                }
            }
        }
    }

    {
        list<Literal*>::iterator effItr = addEffs.begin();
        const list<Literal*>::iterator effEnd = addEffs.end();

        const PropositionAnnotation afterNow(stepID);

        for (; effItr != effEnd; ++effItr) {
            const int litID = (*effItr)->getID();

            bool previouslyDeleted = false;

            {
                const map<int, PropositionAnnotation>::iterator stateItr = theState.retired.find(litID);
                if (stateItr != theState.retired.end()) {
                    assert(theState.first.find(litID) == theState.first.end());
                    previouslyDeleted = true;
                    {
                        const StepAndBeforeOrAfter deletedAt = stateItr->second.negativeAvailableFrom;
                        assert(!deletedAt.never());

                        if (deletedAt.stepID != stepID) {
                            theState.temporalConstraints->addOrdering(deletedAt.stepID, stepID, (deletedAt.beforeOrAfter == StepAndBeforeOrAfter::AFTER));
                        }

                        if (applyDebug) {
                            cout << "\t" << *(*effItr) << " had previously been deleted " << deletedAt;
                        }

                    }

                    PropositionAnnotation & toUpdate = theState.first.insert(*stateItr).first->second;
                    toUpdate.markAsAdded(stepBA);
                    theState.retired.erase(stateItr);

                    if (applyDebug) {
                        cout << ", and is now available from " << stepBA << "\n";
                    }

                }

            }

            if (!previouslyDeleted) {
                const pair<map<int, PropositionAnnotation>::iterator, bool> stateItrPair = theState.first.insert(make_pair(litID, afterNow));

                if (!stateItrPair.second) { // if we haven't just added it (i.e. it was there already)
                    if (stateItrPair.first->second.availableFrom.stepID || stateItrPair.first->second.availableFrom.beforeOrAfter == StepAndBeforeOrAfter::AFTER) {
                        if (applyDebug) {
                            cout << "\t" << *(*effItr) << " used to be available from step " << stateItrPair.first->second.availableFrom.stepID << ", adding ordering constraint\n";
                        }
                        theState.temporalConstraints->addOrdering(stateItrPair.first->second.availableFrom.stepID, stepID, false);
                    }
                    stateItrPair.first->second.availableFrom.stepID = stepID; // override when it's available from
                    stateItrPair.first->second.availableFrom.beforeOrAfter = StepAndBeforeOrAfter::AFTER; // ...  to after now

                }

                if (applyDebug) {
                    cout << "\t" << *(*effItr) << " is brand new and available from " << stepBA << "\n";
                }
            }
        }
    }
}

void POTHelper_updateForEndDeleteInvariantInteractions(MinimalState & theState, const StepAndBeforeOrAfter & endsAt,
        const list<Literal*> & theEffs, const bool & areAdds)
{

    const unsigned int stepID = endsAt.stepID;

    map<int, PropositionAnnotation> & polarised = (areAdds ? theState.retired : theState.first);

#ifndef NDEBUG
    map<int, PropositionAnnotation> & otherpolarised = (areAdds ? theState.first : theState.retired);
#endif

    list<Literal*>::const_iterator effItr = theEffs.begin();
    const list<Literal*>::const_iterator effEnd = theEffs.end();

    for (; effItr != effEnd; ++effItr) {
        if (applyDebug) {
            cout << "\tConsidering end ";
            if (areAdds) {
                cout << "effect " << *(*effItr) << endl;
            } else {
                cout << "effect (not " << *(*effItr) << ")" << endl;
            }
        }

        const int litID = (*effItr)->getID();
        const map<int, PropositionAnnotation>::iterator stateItr = polarised.find(litID);
        if (stateItr != polarised.end()) {

            assert(otherpolarised.find(litID) == otherpolarised.end());

            {
                const map<StepAndBeforeOrAfter, bool> & invAfter = (areAdds ? stateItr->second.addableFrom : stateItr->second.deletableFrom);
                map<StepAndBeforeOrAfter, bool>::const_iterator iamItr = invAfter.begin();
                const map<StepAndBeforeOrAfter, bool>::const_iterator iamEnd = invAfter.end();
                for (; iamItr != iamEnd; ++iamItr) {
                    const StepAndBeforeOrAfter * const iaItr = &(iamItr->first);
                    if (iaItr->stepID != stepID) {
                        if (applyDebug) {
                            cout << "\t - Soonest this effect can happen is ";
                            if (iaItr->beforeOrAfter == StepAndBeforeOrAfter::AFTER) {
                                cout << "epsilon after";
                            } else {
                                cout << "at";
                            }
                            cout << " step " << iaItr->stepID << endl;
                        }
                        theState.temporalConstraints->addOrdering(iaItr->stepID, stepID, (iaItr->beforeOrAfter == StepAndBeforeOrAfter::AFTER));
                    }
                }
            }
            if (areAdds) {
                stateItr->second.promisedAdd.insert(stepID);
            } else {
                stateItr->second.promisedDelete.insert(stepID);
            }
        }
    }
}

void POTHelper_updateForPreconditions(MinimalState & theState, const StepAndBeforeOrAfter & startsAt,
                                      const pair<StepAndBeforeOrAfter, bool> & finishesAt,
                                      const list<Literal*> & reservePositive, const list<Literal*> & reserveNegative)
{

    if (applyDebug) {
        cout << "\tPreconditions requested from " << startsAt << " until " << finishesAt.first << "\n";
    }

    for (int pass = 0; pass < 2; ++pass) {
        const list<Literal*> & reserve = (pass ? reserveNegative : reservePositive);

        map<int, PropositionAnnotation> & polarisedFacts = (pass ? theState.retired : theState.first);

        list<Literal*>::const_iterator precItr = reserve.begin();
        const list<Literal*>::const_iterator precEnd = reserve.end();

        for (; precItr != precEnd; ++precItr) {
            const int litID = (*precItr)->getID();
            map<int, PropositionAnnotation>::iterator stateItr = polarisedFacts.find(litID);

            assert(stateItr != polarisedFacts.end()); // otherwise we're referring to a precondition which isn't true

            // first, we need to make sure that the proposition has been established by the time we try to use it
            // if we want it at the end of the current happening (i.e. as an invariant that is about to start), then
            // it's sufficient for the achiever to fire no later than the current stepID; otherwise,
            // if we want it at the start of the current happening (standard precondition), then
            // we need an epsilon gap if it was added at the end of the achieving happening rather than the start
            // (i.e. true, given the latter isn't valid under the PDDL semantics)

            if (startsAt.beforeOrAfter == StepAndBeforeOrAfter::AFTER) {
                if (stateItr->second.availableFrom.stepID || stateItr->second.availableFrom.beforeOrAfter == StepAndBeforeOrAfter::AFTER) {
                    if (stateItr->second.availableFrom.stepID != startsAt.stepID) { // if it's not just that we want a fact we've just added as an invariant
                        if (applyDebug) {
                            cout << "\tWaiting until " << stateItr->second.availableFrom << " to get " << *(*precItr) << " as an invariant\n";
                        }

                        theState.temporalConstraints->addOrdering(stateItr->second.availableFrom.stepID, startsAt.stepID, false);
                    }
                }
            } else {
                if (stateItr->second.availableFrom.stepID || stateItr->second.availableFrom.beforeOrAfter == StepAndBeforeOrAfter::AFTER) {
                    if (applyDebug) {
                        cout << "\tWaiting until " << stateItr->second.availableFrom << " to get " << *(*precItr) << " as a start/end pre\n";
                    }

                    theState.temporalConstraints->addOrdering(stateItr->second.availableFrom.stepID, startsAt.stepID, stateItr->second.availableFrom.beforeOrAfter == StepAndBeforeOrAfter::AFTER);
                }
            }

            // second, we now need to add the preservation constraints

            if (finishesAt.first.beforeOrAfter == StepAndBeforeOrAfter::AFTER) {
                map<StepAndBeforeOrAfter, bool> & checkInSet = (pass ? stateItr->second.addableFrom : stateItr->second.deletableFrom);
                const StepAndBeforeOrAfter tmp(StepAndBeforeOrAfter::BEFORE, finishesAt.first.stepID);
                const map<StepAndBeforeOrAfter, bool>::iterator checkItr = checkInSet.find(tmp);
                if (checkItr != checkInSet.end()) { // if we already have it as being available from 'before' this step
                    // (i.e. if we had a start prec which is now an over all, or an over all which is now an at end)
                    checkInSet.erase(checkItr); // remove it
                }
                checkInSet.insert(finishesAt); // because we now need to preserve it until after the step
                if (applyDebug) {
                    cout << "\t" << *(*precItr) << " now cannot be ";
                    if (pass) {
                        cout << "added";
                    } else {
                        cout << "deleted";
                    }
                    cout << " until " << finishesAt.first << "\n";
                }
            } else {
                map<StepAndBeforeOrAfter, bool> & checkInSet = (pass ? stateItr->second.addableFrom : stateItr->second.deletableFrom);
                const StepAndBeforeOrAfter tmp(StepAndBeforeOrAfter::AFTER, finishesAt.first.stepID);
                const map<StepAndBeforeOrAfter, bool>::iterator checkItr = checkInSet.find(tmp);
                if (checkItr == checkInSet.end()) { // if we don't have it as being preserved until after this step
                    checkInSet.insert(finishesAt); // add the entry to do so
                }
                if (applyDebug) {
                    cout << "\t" << *(*precItr) << " now cannot be ";
                    if (pass) {
                        cout << "added";
                    } else {
                        cout << "deleted";
                    }
                    cout << " until " << finishesAt.first << "\n";
                }
                if (finishesAt.first.stepID != startsAt.stepID) {

                    // check for clashes with promised deletes
                    const set<int> & promised = (pass ? stateItr->second.promisedAdd : stateItr->second.promisedDelete);

                    set<int>::const_iterator promItr = promised.begin();
                    const set<int>::const_iterator promEnd = promised.end();

                    for (; promItr != promEnd; ++promItr) {
                        theState.temporalConstraints->addOrdering(finishesAt.first.stepID, *promItr, false);
                    }

                }

            }

        }
    }
}

void POTHelper_invariantsCanNowFinish(MinimalState & theState, const StepAndBeforeOrAfter & finishesAt,
                                      const list<Literal*> & reservePositive, const list<Literal*> & reserveNegative)
{

    for (int pass = 0; pass < 2; ++pass) {
        const list<Literal*> & reserve = (pass ? reserveNegative : reservePositive);
        map<int, PropositionAnnotation> & polarisedFacts = (pass ? theState.retired : theState.first);

        list<Literal*>::const_iterator precItr = reserve.begin();
        const list<Literal*>::const_iterator precEnd = reserve.end();

        for (; precItr != precEnd; ++precItr) {
            if (applyDebug) {
                cout << "\tRemoving the invariant ";
                if (pass) cout << "¬";
                cout << *(*precItr) << " started at step " << finishesAt.stepID - 1 << "\n";
            }
            const int litID = (*precItr)->getID();
            map<int, PropositionAnnotation>::iterator stateItr = polarisedFacts.find(litID);

            assert(stateItr != polarisedFacts.end()); // otherwise we're referring to a precondition which isn't true/false

            map<StepAndBeforeOrAfter, bool> & checkInSet = (pass ? stateItr->second.addableFrom : stateItr->second.deletableFrom);

            map<StepAndBeforeOrAfter, bool>::iterator cisItr = checkInSet.find(finishesAt);
            assert(cisItr != checkInSet.end()); // or it means it was never registered as an invariant

            cisItr->second = SAFETOSKIP;

        }
    }
}


void POTHelper_updateForNumericVariables(MinimalState & theState, const int & startsAt, const set<int> & mentioned)
{
    set<int>::iterator cvItr = mentioned.begin();
    const set<int>::iterator cvEnd = mentioned.end();

    for (; cvItr != cvEnd; ++cvItr) {
        const int var = *cvItr;

        if (NumericAnalysis::getDataOnWhichVariablesHaveOrderIndependentEffects()[var]
            && NumericAnalysis::getDominanceConstraints()[var] == NumericAnalysis::E_METRICTRACKING) {            
            continue;
        }
        
        FluentInteraction & currFI = theState.temporalConstraints->lastStepToTouchPNE[var];

        if (currFI.lastInstantaneousEffect != -1 && currFI.lastInstantaneousEffect != startsAt) {
            theState.temporalConstraints->addOrdering(currFI.lastInstantaneousEffect, startsAt, true);
        }

        set<int>::iterator fiItr = currFI.activeCTSEffects.begin();
        const set<int>::iterator fiEnd = currFI.activeCTSEffects.end();

        for (; fiItr != fiEnd; ++fiItr) {
            if (*fiItr != startsAt) {
                theState.temporalConstraints->addOrdering(startsAt, *fiItr, true);
            }

            if (*fiItr != startsAt + 1) {
                theState.temporalConstraints->addOrdering(*fiItr, startsAt + 1, true); // the end of an action is always at the step after it
            }
        }
    }
}

void POTHelper_updateForNumericPreconditions(set<int> & mentioned, list<int> & reserve)
{

    static const int PNEcount = RPGBuilder::getPNECount();

    list<int>::iterator precItr = reserve.begin();
    const list<int>::iterator precEnd = reserve.end();

    for (; precItr != precEnd; ++precItr) {
        RPGBuilder::RPGNumericPrecondition & currPre = RPGBuilder::getNumericPreTable()[*precItr];

        for (int pass = 0; pass < 2; ++pass) {

            const int currVar = (pass ? currPre.RHSVariable : currPre.LHSVariable);

            if (currVar < 0) continue;


            if (currVar < PNEcount) {
                mentioned.insert(currVar);
            } else if (currVar < 2 * PNEcount) {
                mentioned.insert(currVar - PNEcount);
            } else {
                RPGBuilder::ArtificialVariable & currAV = RPGBuilder::getArtificialVariable(currVar);

                for (int i = 0; i < currAV.size; ++i) {
                    const int lv = currAV.fluents[i];
                    if (lv < PNEcount) {
                        mentioned.insert(lv);
                    } else {
                        mentioned.insert(lv - PNEcount);
                    }
                }
            }
        }
    }
}



void POTHelper_updateForDurations(set<int> & mentioned, RPGBuilder::RPGDuration & durationConstraints)
{
    for (int pass = 0; pass < 3; ++pass) {
        const list<RPGBuilder::DurationExpr*> & reserve = durationConstraints[pass];


        list<RPGBuilder::DurationExpr*>::const_iterator precItr = reserve.begin();
        const list<RPGBuilder::DurationExpr*>::const_iterator precEnd = reserve.end();

        for (; precItr != precEnd; ++precItr) {
            mentioned.insert((*precItr)->variables.begin(), (*precItr)->variables.end());
        }
    }

}

void POTHelper_updateForInputsToInstantaneousNumericEffects(set<int> & mentioned, list<int> & reserve)
{

    static const int PNEcount = RPGBuilder::getPNECount();

    list<int>::iterator precItr = reserve.begin();
    const list<int>::iterator precEnd = reserve.end();

    for (; precItr != precEnd; ++precItr) {
        RPGBuilder::RPGNumericEffect & currEff = RPGBuilder::getNumericEff()[*precItr];

        for (int i = 0; i < currEff.size; ++i) {
            const int lv = currEff.variables[i];
            if (lv < 0) continue;
            if (lv < PNEcount) {
                mentioned.insert(lv);
            } else if (lv < 2 * PNEcount) {
                mentioned.insert(lv - PNEcount);
            } else {
                RPGBuilder::ArtificialVariable & currAV = RPGBuilder::getArtificialVariable(lv);

                for (int j = 0; j < currAV.size; ++j) {
                    const int lvtwo = currAV.fluents[j];
                    if (lvtwo < PNEcount) {
                        mentioned.insert(lvtwo);
                    } else {
                        mentioned.insert(lvtwo - PNEcount);
                    }
                }
            }
        }
        if (!currEff.isAssignment) {
            if (currEff.fluentIndex < PNEcount) {
                mentioned.insert(currEff.fluentIndex);
            } else {
                mentioned.insert(currEff.fluentIndex - PNEcount);
            }
        }
    }
}

void POTHelper_updateForOutputsFromInstantaneousNumericEffects(MinimalState & theState, const int & stepID, list<int> & reserve, const int & minDur, const int & maxDur)
{

    static const int PNEcount = RPGBuilder::getPNECount();

    list<int>::iterator precItr = reserve.begin();
    const list<int>::iterator precEnd = reserve.end();

    list<pair<int, pair<double, double> > > updated;

    for (; precItr != precEnd; ++precItr) {
        RPGBuilder::RPGNumericEffect & currEff = RPGBuilder::getNumericEff()[*precItr];

        if (NumericAnalysis::getDataOnWhichVariablesHaveOrderIndependentEffects()[currEff.fluentIndex]
            && NumericAnalysis::getDominanceConstraints()[currEff.fluentIndex] == NumericAnalysis::E_METRICTRACKING) {
            continue;
        }                                
        
        updated.push_back(pair<int, pair<double, double> >(currEff.fluentIndex, currEff.applyEffectMinMax(theState.secondMin, theState.secondMax, minDur, maxDur)));

        const int localFluentIndex = (currEff.fluentIndex < PNEcount ? currEff.fluentIndex : currEff.fluentIndex - PNEcount);

        theState.temporalConstraints->lastStepToTouchPNE[localFluentIndex].lastInstantaneousEffect = stepID;

        {

            map<int, int> & invSet = theState.temporalConstraints->lastStepToTouchPNE[localFluentIndex].activeInvariants;

            map<int, int>::iterator isItr = invSet.begin();
            const map<int, int>::iterator isEnd = invSet.end();

            for (; isItr != isEnd; ++isItr) {
                if (stepID != isItr->second) {
                    if (applyDebug) {
                        cout << "\tRequesting ordering to come after " << isItr->second << ", the start of an invariant on " << *(RPGBuilder::getPNE(currEff.fluentIndex)) << "\n";
                    }
                    theState.temporalConstraints->addOrdering(isItr->second, stepID, true);
                }

                if (stepID != isItr->first) {
                    cout << "\tRequesting ordering to come after " << isItr->second << ", the end of an invariant on " << *(RPGBuilder::getPNE(currEff.fluentIndex)) << "\n";
                    theState.temporalConstraints->addOrdering(stepID, isItr->first, true);
                }
            }
        }

        {

            set<int> & invSet = theState.temporalConstraints->lastStepToTouchPNE[localFluentIndex].activeCTSEffects;

            set<int>::iterator isItr = invSet.begin();
            const set<int>::iterator isEnd = invSet.end();

            for (; isItr != isEnd; ++isItr) {
                if (stepID != (*isItr - 1)) {
                    theState.temporalConstraints->addOrdering(*isItr - 1, stepID, true);
                }

                if (stepID != *isItr) {
                    theState.temporalConstraints->addOrdering(stepID, *isItr, true);
                }
            }
        }

    }

    list<pair<int, pair<double, double> > >::iterator updItr = updated.begin();
    const list<pair<int, pair<double, double> > >::iterator updEnd = updated.end();

    for (; updItr != updEnd; ++updItr) {
        theState.secondMin[updItr->first] = updItr->second.first;
        theState.secondMax[updItr->first] = updItr->second.second;
    }


}

void POTHelper_registerContinuousNumericEffects(MinimalState & theState, const int & startStepID, const int & endStepID, RPGBuilder::LinearEffects* const effs, const bool & registerNotDeregister)
{

    if (!effs) return;

    const unsigned int lim = effs->vars.size();

    for (unsigned int v = 0; v < lim; ++v) {
        
        if (NumericAnalysis::getDataOnWhichVariablesHaveOrderIndependentEffects()[effs->vars[v]]
            && NumericAnalysis::getDominanceConstraints()[effs->vars[v]] == NumericAnalysis::E_METRICTRACKING) {
            continue;
        }
                
                
        
        FluentInteraction & currFI = theState.temporalConstraints->lastStepToTouchPNE[effs->vars[v]];
        if (registerNotDeregister) {
            currFI.lastInstantaneousEffect = startStepID;
            currFI.activeCTSEffects.insert(endStepID);
        } else {
            currFI.activeCTSEffects.erase(endStepID);
        }

        {
            const int stepID = (registerNotDeregister ? startStepID : endStepID);

            map<int, int> & invSet = theState.temporalConstraints->lastStepToTouchPNE[effs->vars[v]].activeInvariants;

            map<int, int>::iterator isItr = invSet.begin();
            const map<int, int>::iterator isEnd = invSet.end();

            for (; isItr != isEnd; ++isItr) {
                if (stepID != isItr->second) {
                    theState.temporalConstraints->addOrdering(isItr->second, stepID, true);
                }

                if (stepID != isItr->first) {
                    theState.temporalConstraints->addOrdering(stepID, isItr->first, true);
                }
            }
        }
    }
}

void POTHelper_registerInvariantNumerics(MinimalState & theState, const int & startStepID, const int & endStepID, list<int> & reserve, const bool & registerNotDeregister)
{
    static const int PNEcount = RPGBuilder::getPNECount();

    if (applyDebug) {
        if (registerNotDeregister) {
            cout << "\tNumeric invariants requested from " << startStepID << " until " << endStepID << "\n";
        } else {
            cout << "\tNumeric invariants expired from " << startStepID << " until " << endStepID << "\n";
        }
    }


    list<int>::iterator precItr = reserve.begin();
    const list<int>::iterator precEnd = reserve.end();

    set<int> mentioned;

    for (; precItr != precEnd; ++precItr) {
        RPGBuilder::RPGNumericPrecondition & currPre = RPGBuilder::getNumericPreTable()[*precItr];

        for (int pass = 0; pass < 2; ++pass) {

            const int currVar = (pass ? currPre.RHSVariable : currPre.LHSVariable);

            if (currVar < 0) continue;


            if (currVar < PNEcount) {
                mentioned.insert(currVar);
                if (registerNotDeregister) {
                    theState.temporalConstraints->lastStepToTouchPNE[currVar].activeInvariants.insert(make_pair(endStepID, startStepID));
                } else {
                    theState.temporalConstraints->lastStepToTouchPNE[currVar].activeInvariants.erase(endStepID);
                }
            } else if (currVar < 2 * PNEcount) {
                mentioned.insert(currVar - PNEcount);
                if (registerNotDeregister) {
                    INVARIANTINSERT(theState.temporalConstraints->lastStepToTouchPNE[currVar - PNEcount].activeInvariants, make_pair(endStepID, startStepID), theState.planLength);
                } else {
                    INVARIANTERASE(theState.temporalConstraints->lastStepToTouchPNE[currVar - PNEcount].activeInvariants, endStepID, theState.planLength);
                }
            } else {
                RPGBuilder::ArtificialVariable & currAV = RPGBuilder::getArtificialVariable(currVar);

                for (int i = 0; i < currAV.size; ++i) {
                    const int lv = currAV.fluents[i];
                    if (lv < PNEcount) {
                        mentioned.insert(lv);
                        if (registerNotDeregister) {
                            INVARIANTINSERT(theState.temporalConstraints->lastStepToTouchPNE[lv].activeInvariants, make_pair(endStepID, startStepID), theState.planLength);
                        } else {
                            INVARIANTERASE(theState.temporalConstraints->lastStepToTouchPNE[lv].activeInvariants, endStepID, theState.planLength);
                        }
                    } else {
                        mentioned.insert(lv - PNEcount);
                        if (registerNotDeregister) {
                            INVARIANTINSERT(theState.temporalConstraints->lastStepToTouchPNE[lv-PNEcount].activeInvariants, make_pair(endStepID, startStepID), theState.planLength);
                        } else {
                            INVARIANTERASE(theState.temporalConstraints->lastStepToTouchPNE[lv-PNEcount].activeInvariants, endStepID, theState.planLength);
                        }
                    }
                }
            }
        }
    }

    if (registerNotDeregister) {
        POTHelper_updateForNumericVariables(theState, startStepID, mentioned);
    } else {
        POTHelper_updateForNumericVariables(theState, startStepID, mentioned);
    }

}

void sanityCheck(MinimalState & workOn)
{

    if (!workOn.startedActions.empty()) return;

    // for now the only check is that we don't have any hanging deletable froms if all actions have finished

    for (int pass = 0; pass < 2; ++pass) {

        map<int, PropositionAnnotation> & polarised = (pass ? workOn.retired : workOn.first);

        map<int, PropositionAnnotation>::iterator propItr = polarised.begin();
        const map<int, PropositionAnnotation>::iterator propEnd = polarised.end();

        for (; propItr != propEnd; ++propItr) {
            map<StepAndBeforeOrAfter, bool> & deletableFrom = (pass ? propItr->second.addableFrom : propItr->second.deletableFrom);
            map<StepAndBeforeOrAfter, bool>::iterator dfItr = deletableFrom.begin();
            const map<StepAndBeforeOrAfter, bool>::iterator dfEnd = deletableFrom.end();

            for (; dfItr != dfEnd; ++dfItr) {
                assert(dfItr->second == SAFETOSKIP);
            }
        }
    }

}

static unsigned int oldStepCount;

MinimalState & PartialOrderTransformer::applyAction(MinimalState & theStateHidden, const ActionSegment & a, const bool & inPlace, const double & minDur, const double & maxDur)
{
	//std::cout << "[PartialOrderTransformer::applyAction] " << std::endl;
	//std::cout << "The hidden state " << std::endl;
	//theStateHidden.printState(std::cout);
	//std::cout << "The action segment" << std::endl;
	//a.first->write(std::cout);
	//std::cout << std::endl;
	applyDebug = Globals::globalVerbosity & 1048576;

	unsigned int extensionNeeded = 0;

	if (applyDebug) {
		oldStepCount = theStateHidden.temporalConstraints->size();
		cout << "Applying action.  Previously had space for constraints on " << oldStepCount << " steps\n";
		assert(oldStepCount == theStateHidden.planLength);
	}


	if (a.first) {
		if (a.second == VAL::E_AT_START) {
				++extensionNeeded;
				const int actID = a.first->getID();

				if (!RPGBuilder::getRPGDEs(actID).empty()) {
					++extensionNeeded; // isn't a non-temporal action
					if (applyDebug) cout << "Temporal record extension of two needed - starting " << *(a.first) << "\n";
				} else {
					if (applyDebug) cout << "Temporal record extension of one needed - applying an instantaneous action, " << *(a.first) << "\n";
				}
		} else {
				if (applyDebug) cout << "Temporal record extension of zero needed - is the end of " << *(a.first) << "\n";
		}
	} else {
		extensionNeeded = (a.divisionID - theStateHidden.nextTIL) + 1;
		if (applyDebug) {
				cout << "Temporal record extension of " << extensionNeeded << " needed - applying TIL " << a.divisionID;
				if (a.divisionID != theStateHidden.nextTIL) {
					cout << "; next one would be " << theStateHidden.nextTIL;
				}
				cout << "\n";
		}
	}
	MinimalState * const workOn = (inPlace ? &theStateHidden : new MinimalState(theStateHidden, extensionNeeded));

	if (inPlace && extensionNeeded) {
		theStateHidden.temporalConstraints->extend(extensionNeeded);
	}

	if (applyDebug) {
		const unsigned int newStepCount = workOn->temporalConstraints->size();
		cout << "Now have space for constraints on " << newStepCount << " steps\n";
		assert(newStepCount - oldStepCount == extensionNeeded);
	}

	if (!a.first) { // applying a TIL
		static vector<RPGBuilder::FakeTILAction*> & tilVec = RPGBuilder::getTILVec();
			std::cout << "[PartialOrderTransformer::applyAction] applying TIL." << std::endl;
		for (; workOn->nextTIL <= a.divisionID; ++(workOn->nextTIL)) {

				POTHelper_updateForInstantaneousEffects(*workOn,  StepAndBeforeOrAfter(StepAndBeforeOrAfter::AFTER, workOn->planLength), tilVec[workOn->nextTIL]->delEffects, tilVec[workOn->nextTIL]->addEffects);
				++workOn->planLength;
		}

		workOn->temporalConstraints->setMostRecentStep(workOn->planLength - 1);

		return *workOn;
	}

	const int actID = a.first->getID();

	if (a.second == VAL::E_AT_START) {

		if (RPGBuilder::getRPGDEs(actID).empty()) { // non-temporal action
				list<Literal*> & pres = RPGBuilder::getStartPropositionalPreconditions()[actID];
				list<Literal*> & negpres = RPGBuilder::getStartNegativePropositionalPreconditions()[actID];
				POTHelper_updateForPreconditions(*workOn, StepAndBeforeOrAfter(StepAndBeforeOrAfter::BEFORE, workOn->planLength),
															make_pair(StepAndBeforeOrAfter(StepAndBeforeOrAfter::AFTER, workOn->planLength), SAFETOSKIP),
															pres, negpres);

				list<Literal*> & delEffs = RPGBuilder::getStartPropositionDeletes()[actID];
				list<Literal*> & addEffs = RPGBuilder::getStartPropositionAdds()[actID];
				//std::cout << "[PartialOrderTransformer::applyAction] applying non-temporal effects." << std::endl;
				POTHelper_updateForInstantaneousEffects(*workOn, StepAndBeforeOrAfter(StepAndBeforeOrAfter::AFTER, workOn->planLength), delEffs, addEffs);

				{
					set<int> mentioned;
					POTHelper_updateForNumericPreconditions(mentioned, RPGBuilder::getStartPreNumerics()[actID]);
					POTHelper_updateForInputsToInstantaneousNumericEffects(mentioned, RPGBuilder::getStartEffNumerics()[actID]);

					POTHelper_updateForNumericVariables(*workOn, workOn->planLength, mentioned);
				}

				POTHelper_updateForOutputsFromInstantaneousNumericEffects(*workOn, workOn->planLength, RPGBuilder::getStartEffNumerics()[actID], minDur, maxDur);
				POTHelper_updateForPathfinder(*workOn, a, StepAndBeforeOrAfter(StepAndBeforeOrAfter::AFTER, workOn->planLength));
				workOn->temporalConstraints->setMostRecentStep(workOn->planLength);

				++workOn->planLength;

				//if (applyDebug) sanityCheck(*workOn);

				//return *workOn;
		}
		else
		{


			const bool skip = (RPGBuilder::canSkipToEnd(actID) ? SAFETOSKIP : UNSAFETOSKIP);

			workOn->temporalConstraints->setMostRecentStep(workOn->planLength);

			const int startStepID = workOn->planLength++;
			const int endStepID = workOn->planLength++;

			if (applyDebug) {
					assert(workOn->planLength == workOn->temporalConstraints->size());
					cout << "New step IDs: " << startStepID << "," << endStepID << "\n";
			}

			workOn->temporalConstraints->addOrdering(startStepID, endStepID, false);

			set<int> mentioned;

			POTHelper_updateForDurations(mentioned, *(RPGBuilder::getRPGDEs(actID)[0]));

			{
					if (applyDebug) cout << " * Requesting start preconditions\n";
					list<Literal*> & pres = RPGBuilder::getStartPropositionalPreconditions()[actID];
					list<Literal*> & negpres = RPGBuilder::getStartNegativePropositionalPreconditions()[actID];
					POTHelper_updateForPreconditions(*workOn, StepAndBeforeOrAfter(StepAndBeforeOrAfter::BEFORE, startStepID),
																make_pair(StepAndBeforeOrAfter(StepAndBeforeOrAfter::AFTER, startStepID), SAFETOSKIP),
																pres, negpres);

					POTHelper_updateForNumericPreconditions(mentioned, RPGBuilder::getStartPreNumerics()[actID]);
			}

			{

					if (applyDebug) cout << " * Applying start effects\n";

					list<Literal*> & delEffs = RPGBuilder::getStartPropositionDeletes()[actID];
					list<Literal*> & addEffs = RPGBuilder::getStartPropositionAdds()[actID];

					std::cout << "[PartialOrderTransformer::applyAction] applying Start effects." << std::endl;
					POTHelper_updateForInstantaneousEffects(*workOn, StepAndBeforeOrAfter(StepAndBeforeOrAfter::AFTER, startStepID), delEffs, addEffs);
			}

			{
					POTHelper_updateForInputsToInstantaneousNumericEffects(mentioned, RPGBuilder::getStartEffNumerics()[actID]);
					POTHelper_updateForNumericVariables(*workOn, startStepID, mentioned);

					mentioned.clear();

					POTHelper_updateForOutputsFromInstantaneousNumericEffects(*workOn, startStepID, RPGBuilder::getStartEffNumerics()[actID], minDur, maxDur);
					POTHelper_registerContinuousNumericEffects(*workOn, startStepID, endStepID, RPGBuilder::getLinearDiscretisation()[actID], true);

			}

			{
					if (applyDebug) cout << " * Requesting invariants from " << startStepID << " to " << endStepID << "\n";
					list<Literal*> & pres = RPGBuilder::getInvariantPropositionalPreconditions()[actID];
					list<Literal*> & negpres = RPGBuilder::getInvariantNegativePropositionalPreconditions()[actID];
					POTHelper_updateForPreconditions(*workOn, StepAndBeforeOrAfter(StepAndBeforeOrAfter::AFTER, startStepID),
																make_pair(StepAndBeforeOrAfter(StepAndBeforeOrAfter::BEFORE, endStepID), skip),
																pres, negpres);
					POTHelper_registerInvariantNumerics(*workOn, startStepID, endStepID, RPGBuilder::getInvariantNumerics()[actID], true);
			}

			if (skip == SAFETOSKIP) {

					{
						
						if (applyDebug) cout << " * Compression-safe action - requesting end preconditions\n";
						list<Literal*> & pres = RPGBuilder::getEndPropositionalPreconditions()[actID];
						list<Literal*> & negpres = RPGBuilder::getEndNegativePropositionalPreconditions()[actID];
						POTHelper_updateForPreconditions(*workOn, StepAndBeforeOrAfter(StepAndBeforeOrAfter::BEFORE, endStepID),
																	make_pair(StepAndBeforeOrAfter(StepAndBeforeOrAfter::AFTER, endStepID), skip),
																	pres, negpres);
																	
					}
					
					{
						if (applyDebug) cout << " * Compression-safe action - recording end add effects\n";

						static list<Literal*> delEffs;
						list<Literal*> & addEffs = RPGBuilder::getEndPropositionAdds()[actID];

						std::cout << "[PartialOrderTransformer::applyAction] applying end affects." << std::endl;
						POTHelper_updateForInstantaneousEffects(*workOn, StepAndBeforeOrAfter(StepAndBeforeOrAfter::AFTER, endStepID), delEffs, addEffs);
					}

					++(workOn->actionsExecuting);

			} else {
					{
						if (applyDebug) cout << " * Non-compression-safe action - avoiding end-delete--invariant clashes\n";

						list<Literal*> & delEffs = RPGBuilder::getEndPropositionDeletes()[actID];
						POTHelper_updateForEndDeleteInvariantInteractions(*workOn, StepAndBeforeOrAfter(StepAndBeforeOrAfter::AFTER, endStepID), delEffs, false);

						list<Literal*> & addEffs = RPGBuilder::getEndPropositionAdds()[actID];
						POTHelper_updateForEndDeleteInvariantInteractions(*workOn, StepAndBeforeOrAfter(StepAndBeforeOrAfter::AFTER, endStepID), addEffs, true);

					}

					workOn->startedActions[actID].insert(endStepID);
					++(workOn->actionsExecuting);

					// we only need record the action as started in the non-compression safe case
					// as it end needs not be explicitly applied otherwise
			}
		}

		///if (applyDebug) sanityCheck(*workOn);

		///return *workOn;
	}

	// otherwise, we're ending a non-compression-safe action
	else
	{
	map<int, set<int> >::iterator saItr = workOn->startedActions.find(actID);

	assert(saItr != workOn->startedActions.end());

	set<int>::iterator endItr = saItr->second.begin();

	const int endStepID = *endItr;
	const int startStepID = endStepID - 1;
	saItr->second.erase(endItr);
	if (saItr->second.empty()) workOn->startedActions.erase(saItr);

	--(workOn->actionsExecuting);

	workOn->temporalConstraints->setMostRecentStep(endStepID);

	{
		if (applyDebug) cout << " * De-registering invariants\n";

		POTHelper_invariantsCanNowFinish(*workOn, StepAndBeforeOrAfter(StepAndBeforeOrAfter::BEFORE, endStepID),
													RPGBuilder::getInvariantPropositionalPreconditions()[actID],
													RPGBuilder::getInvariantNegativePropositionalPreconditions()[actID]);
		POTHelper_registerInvariantNumerics(*workOn, startStepID, endStepID, RPGBuilder::getInvariantNumerics()[actID], false);
		POTHelper_registerContinuousNumericEffects(*workOn, -1, endStepID, RPGBuilder::getLinearDiscretisation()[actID], false);
	}

	{
		if (applyDebug) cout << " * Requesting end preconditions\n";
		list<Literal*> & pres = RPGBuilder::getEndPropositionalPreconditions()[actID];
		list<Literal*> & negpres = RPGBuilder::getEndNegativePropositionalPreconditions()[actID];
		POTHelper_updateForPreconditions(*workOn, StepAndBeforeOrAfter(StepAndBeforeOrAfter::BEFORE, endStepID),
													make_pair(StepAndBeforeOrAfter(StepAndBeforeOrAfter::AFTER, endStepID), SAFETOSKIP),
													pres, negpres);
	}


	{
		if (applyDebug) cout << " * Recording end effects\n";
		list<Literal*> & delEffs = RPGBuilder::getEndPropositionDeletes()[actID];
		list<Literal*> & addEffs = RPGBuilder::getEndPropositionAdds()[actID];

		std::cout << "[PartialOrderTransformer::applyAction] Recording end effects." << std::endl;
		POTHelper_updateForInstantaneousEffects(*workOn, StepAndBeforeOrAfter(StepAndBeforeOrAfter::AFTER, endStepID), delEffs, addEffs);
	}

	{
		set<int> mentioned;
		POTHelper_updateForNumericPreconditions(mentioned, RPGBuilder::getEndPreNumerics()[actID]);
		POTHelper_updateForInputsToInstantaneousNumericEffects(mentioned, RPGBuilder::getEndEffNumerics()[actID]);
		POTHelper_updateForNumericVariables(*workOn, endStepID, mentioned);

		POTHelper_updateForOutputsFromInstantaneousNumericEffects(*workOn, endStepID, RPGBuilder::getEndEffNumerics()[actID], minDur, maxDur);

	}
	}
	
	if (applyDebug) sanityCheck(*workOn);

	return *workOn;


};



};

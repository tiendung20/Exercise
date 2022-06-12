//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
// 

#include "RSUControlApp.h"
#include "Constant.h"
#include "Graph.h"
#include <vector>
#include <string>

#include "veins/modules/application/traci/TraCIDemo11pMessage_m.h"

using namespace veins;

Register_Class(RSUControlApp);

void printVertex(NodeVertex *root) {
    if (root != NULL) {
        printVertex(root->left);
        EV << root->v->getId() << endl;
        for (auto r : root->v->getFrom()) {
            EV << r->getId() << " - " << r->getW() << " ";
        }
        EV << endl;
        for (auto r : root->v->getTo()) {
            EV << r->getId() << " - " << r->getW() << " ";
        }
        EV << endl;
        if (root->j_of_vertex.size() > 2) {
            for (auto r : root->v->getInternals()) {
                if (r->getFrom().length() > 0 && r->getTo().length() > 0)
                    EV << r->getFrom() << " + " << r->getTo() << " + "
                              << r->getW() << endl;
            }
        }
        printVertex(root->right);
    }
}

void RSUControlApp::initialize(int stage) {
    DemoBaseApplLayer::initialize(stage);
    if (stage == 0) {
        sendBeacon = new cMessage("send Beacon");
        graph = new Graph();
        count = new Count();
        count->k = 0;
        count->i = 0;
        count->laneId = "";
//        printVertex(graph->getVertex());
    } else if (stage == 1) {
        // Initializing members that require initialized other modules goes here
    }

}

void RSUControlApp::finish() {
    //Duoc goi khi RSU ket thuc cong viec
    DemoBaseApplLayer::finish();
    EV << "Finish here" << endl;
    for (auto i : message) {
        EV << i << endl;
    }
    // statistics recording goes here
}

void RSUControlApp::onBSM(DemoSafetyMessage *bsm) {
    //for my own simulation circle
}

void RSUControlApp::onWSM(BaseFrame1609_4 *wsm) {
    // Your application has received a data message from another car or RSU
    // code for handling the message goes here, see TraciDemo11p.cc for examples
    cPacket *enc = wsm->getEncapsulatedPacket();
    if (TraCIDemo11pMessage *bc = dynamic_cast<TraCIDemo11pMessage*>(enc)) {
//        if(strcmp(Constant::FIRST, bc->getDemoData()) == 0){
//        EV << "my message = " << bc->getDemoData() << " from "
//                  << bc->getSenderAddress() << endl;
        if (sendBeacon != NULL) {
            if (sendBeacon->isScheduled()) {
                cancelEvent(sendBeacon);
            }

            TraCIDemo11pMessage *rsuBeacon = new TraCIDemo11pMessage();

            char *ret = mergeContent(bc->getSenderAddress());
            rsuBeacon->setDemoData(ret);
            rsuBeacon->setSenderAddress(myId);
            BaseFrame1609_4 *WSM = new BaseFrame1609_4();
            WSM->encapsulate(rsuBeacon);
            populateWSM(WSM);
            send(WSM, lowerLayerOut);
        }
        std::stringstream streamData(bc->getDemoData());
        std::string str, before_name;
//        AGV *cur = NULL;
//        for (auto a : vh) {
//            if (a->id == std::to_string(bc->getSenderAddress()))
//                cur = a;
//        }
//        if (cur == NULL) {
//            cur = new AGV();
//            cur->id = std::to_string(bc->getSenderAddress());
//            cur->n = new Count();
//            vh.push_back(cur);
//        }
        before_name = count->laneId;
        int i = 0;
//        while (getline(streamData, str, ' ')) {
//            if (i == 0) {
//                str.erase(str.find("_"));
//                if (cur->n->laneId.length() == 0) {
//                    cur->n->laneId = str;
//                } else if (cur->n->laneId.length() > 0
//                        && str != cur->n->laneId) {
////                    EV << count->laneId << " " << count->k << endl;
//                    std::string mes;
//                    if (before_name.front() == ':') {
//                        mes = cur->n->laneId + " "
//                                + std::to_string(cur->n->k * 0.1);
//                        NodeVertex *nv = graph->searchVertex(before_name);
//                        double w =
//                                (nv->v->getW() == 0) ?
//                                        cur->n->k :
//                                        (nv->v->getW() + cur->n->k) / 2;
//                        nv->v->setW(w);
//                    } else {
//                        std::string full_name = before_name + "-" + str;
//                        mes = full_name + " " + std::to_string(cur->n->k * 0.1);
//                        NodeVertex *nv = graph->searchVertex(full_name);
//                        if (nv != NULL) {
//                            double w =
//                                    (nv->v->getW() == 0) ?
//                                            cur->n->k :
//                                            (nv->v->getW() + cur->n->k) / 2;
//                            nv->v->setW(w);
//                        } else {
//                            Vertex *e = new Vertex();
//                            cur->n->k = 0;             e->setId(full_name);
//                            e->setW(cur->n->k);
//                            graph->addVertex(e);
//                        }
//                    }
//                    message.push_back(mes);
//                    cur->n->i = cur->n->k = 0;
//                    cur->n->laneId.erase();
//                }
//            } else if (i == 2) {
//                if (std::stod(str) == 0) {
//                    cur->n->i = 0;
//                    cur->n->k++;
//                } else {
//                    if (cur->n->i == 0 && cur->n->k > 0)
//                        cur->n->k--;
//                    cur->n->i++;
//                }
//            }
//            i++;
//        }
        while (getline(streamData, str, ' ')) {
            if (i == 0) {
                str.erase(str.find("_"));
                if (count->laneId.length() == 0) {
                    count->laneId = str;
                } else if (count->laneId.length() > 0 && str != count->laneId) {
//                    EV << count->laneId << " " << count->k << endl;
                    std::string mes;
                    if (before_name.front() == ':') {
                        mes = count->laneId + " "
                                + std::to_string(count->k * 0.1);
                        NodeVertex *nv = graph->searchVertex(before_name);
                        double w =
                                (nv->v->getW() == 0) ?
                                        count->k :
                                        (nv->v->getW() + count->k) / 2;
                        nv->v->setW(w);
                    } else {
                        std::string full_name = before_name + "-" + str;
                        mes = full_name + " " + std::to_string(count->k * 0.1);
                        NodeVertex *nv = graph->searchVertex(full_name);
                        if (nv != NULL) {
                            double w =
                                    (nv->v->getW() == 0) ?
                                            count->k :
                                            (nv->v->getW() + count->k) / 2;
                            nv->v->setW(w);
                        } else {
                            Vertex *e = new Vertex();
                            e->setId(full_name);
                            e->setW(count->k);
                            graph->addVertex(e);
                        }
                    }
                    message.push_back(mes);
                    count->i = 0;
                    count->k = 0;
                    count->laneId.erase();
                }
            } else if (i == 2) {
                if (std::stod(str) == 0) {
                    count->i = 0;
                    count->k++;
                } else {
                    if (count->i == 0 && count->k > 0)
                        count->k--;
                    count->i++;
                }
            }
            i++;
        }
//    }
    }
}

void RSUControlApp::onWSA(DemoServiceAdvertisment *wsa) {
// Your application has received a service advertisement from another car or RSU
// code for handling the message goes here, see TraciDemo11p.cc for examples
}

void RSUControlApp::handleSelfMsg(cMessage *msg) {
    DemoBaseApplLayer::handleSelfMsg(msg);
}

void RSUControlApp::handlePositionUpdate(cObject *obj) {
    DemoBaseApplLayer::handlePositionUpdate(obj);
// the vehicle has moved. Code that reacts to new positions goes here.
// member variables such as currentPosition and currentSpeed are updated in the parent class

}


//
// Created by michalnowicki on 17.10.17.
//

#include "HighGradientPoint.h"

namespace ORB_SLAM2 {
    long unsigned int HighGradientPoint::nextId=0;

    HighGradientPoint::HighGradientPoint(double u, double v, double invDepth) :
            u(u), v(v), invDepth(invDepth) {
        id = nextId++;
    }

    cv::Mat HighGradientPoint::getGlobalPosition()  {
        cv::Mat point3D = cv::Mat(3, 1, CV_32F);
        point3D.at<float>(2) = 1. / invDepth;
        point3D.at<float>(0) = (u - refKF->cx) * point3D.at<float>(2) / refKF->fx;
        point3D.at<float>(1) = (v - refKF->cy) * point3D.at<float>(2) / refKF->fy;

        cv::Mat worldPointPos =
                refKF->GetRotation().t() * point3D -
                refKF->GetRotation().t() * refKF->GetTranslation();
        return worldPointPos;
    }

    void ExtractorNodeHG::DivideNode(ExtractorNodeHG &n1, ExtractorNodeHG &n2, ExtractorNodeHG &n3, ExtractorNodeHG &n4)
    {
        const int halfX = ceil(static_cast<float>(UR.x-UL.x)/2);
        const int halfY = ceil(static_cast<float>(BR.y-UL.y)/2);

        //Define boundaries of childs
        n1.UL = UL;
        n1.UR = cv::Point2i(UL.x+halfX,UL.y);
        n1.BL = cv::Point2i(UL.x,UL.y+halfY);
        n1.BR = cv::Point2i(UL.x+halfX,UL.y+halfY);
        n1.vKeys.reserve(vKeys.size());

        n2.UL = n1.UR;
        n2.UR = UR;
        n2.BL = n1.BR;
        n2.BR = cv::Point2i(UR.x,UL.y+halfY);
        n2.vKeys.reserve(vKeys.size());

        n3.UL = n1.BL;
        n3.UR = n1.BR;
        n3.BL = BL;
        n3.BR = cv::Point2i(n1.BR.x,BL.y);
        n3.vKeys.reserve(vKeys.size());

        n4.UL = n3.UR;
        n4.UR = n2.BR;
        n4.BL = n3.BR;
        n4.BR = BR;
        n4.vKeys.reserve(vKeys.size());

        //Associate points to childs
        for(size_t i=0;i<vKeys.size();i++)
        {
            HighGradientPoint * kp = vKeys[i];
            if(kp->u <n1.UR.x)
            {
                if(kp->v <n1.BR.y)
                    n1.vKeys.push_back(kp);
                else
                    n3.vKeys.push_back(kp);
            }
            else if(kp->v <n1.BR.y)
                n2.vKeys.push_back(kp);
            else
                n4.vKeys.push_back(kp);
        }

        if(n1.vKeys.size()==1)
            n1.bNoMore = true;
        if(n2.vKeys.size()==1)
            n2.bNoMore = true;
        if(n3.vKeys.size()==1)
            n3.bNoMore = true;
        if(n4.vKeys.size()==1)
            n4.bNoMore = true;

    }

    list<HighGradientPoint*> HighGradientPoint::DistributeOctTree(KeyFrame* currentKF,
                                                                  list<HighGradientPoint*> vToDistributeKeys, const int &minX,
                                                                  const int &maxX, const int &minY, const int &maxY, const int &numberOfFeatures)
    {

        double cx = currentKF->cx, cy = currentKF->cy;
        double fx = currentKF->fx, fy = currentKF->fy;

        list<HighGradientPoint*> outsideCurImage;
        for (auto it = vToDistributeKeys.begin(); it!= vToDistributeKeys.end();) {
            cv::Mat worldPos = (*it)->getGlobalPosition();
            cv::Mat posInCurrent = currentKF->GetRotation() * worldPos + currentKF->GetTranslation();
            double X = posInCurrent.at<float>(0);
            double Y = posInCurrent.at<float>(1);
            double Z = posInCurrent.at<float>(2);
            (*it)->curKF_u = fx * X / Z + cx;
            (*it)->curKF_v = fy * Y / Z + cy;

            // Projection outside the image
            if ( (*it)->curKF_u < minX || (*it)->curKF_u >= maxX || (*it)->curKF_v < minY || (*it)->curKF_v >= maxY ) {
                outsideCurImage.push_back(*it);
                it = vToDistributeKeys.erase(it);
            }
            else
                it++;
        }
        const int N = numberOfFeatures - outsideCurImage.size();


        // Compute how many initial nodes
        const int nIni = round(static_cast<float>(maxX-minX)/(maxY-minY));

        const float hX = static_cast<float>(maxX-minX)/nIni;

        list<ExtractorNodeHG> lNodes;

        vector<ExtractorNodeHG*> vpIniNodes;
        vpIniNodes.resize(nIni);

        for(int i=0; i<nIni; i++)
        {
            ExtractorNodeHG ni;
            ni.UL = cv::Point2i(hX*static_cast<float>(i),0);
            ni.UR = cv::Point2i(hX*static_cast<float>(i+1),0);
            ni.BL = cv::Point2i(ni.UL.x,maxY-minY);
            ni.BR = cv::Point2i(ni.UR.x,maxY-minY);
            ni.vKeys.reserve(vToDistributeKeys.size());

            lNodes.push_back(ni);
            vpIniNodes[i] = &lNodes.back();
        }

        //Associate points to childs
        for (auto &kp : vToDistributeKeys) {
//            std::cout << "vpIniNodes.size() = " << vpIniNodes.size() << " kp->u = " << kp->u << " hX = " << hX << std::endl;
            vpIniNodes[kp->u/hX]->vKeys.push_back(kp);
        }

        list<ExtractorNodeHG>::iterator lit = lNodes.begin();

        while(lit!=lNodes.end())
        {
            if(lit->vKeys.size()==1)
            {
                lit->bNoMore=true;
                lit++;
            }
            else if(lit->vKeys.empty())
                lit = lNodes.erase(lit);
            else
                lit++;
        }

        bool bFinish = false;

        int iteration = 0;

        vector<pair<int,ExtractorNodeHG*> > vSizeAndPointerToNode;
        vSizeAndPointerToNode.reserve(lNodes.size()*4);


        while(!bFinish)
        {
            iteration++;

            int prevSize = lNodes.size();

            lit = lNodes.begin();

            int nToExpand = 0;

            vSizeAndPointerToNode.clear();

            while(lit!=lNodes.end())
            {
                if(lit->bNoMore)
                {
                    // If node only contains one point do not subdivide and continue
                    lit++;
                    continue;
                }
                else
                {
                    // If more than one point, subdivide
                    ExtractorNodeHG n1,n2,n3,n4;
                    lit->DivideNode(n1,n2,n3,n4);

                    // Add childs if they contain points
                    if(n1.vKeys.size()>0)
                    {
                        lNodes.push_front(n1);
                        if(n1.vKeys.size()>1)
                        {
                            nToExpand++;
                            vSizeAndPointerToNode.push_back(make_pair(n1.vKeys.size(),&lNodes.front()));
                            lNodes.front().lit = lNodes.begin();
                        }
                    }
                    if(n2.vKeys.size()>0)
                    {
                        lNodes.push_front(n2);
                        if(n2.vKeys.size()>1)
                        {
                            nToExpand++;
                            vSizeAndPointerToNode.push_back(make_pair(n2.vKeys.size(),&lNodes.front()));
                            lNodes.front().lit = lNodes.begin();
                        }
                    }
                    if(n3.vKeys.size()>0)
                    {
                        lNodes.push_front(n3);
                        if(n3.vKeys.size()>1)
                        {
                            nToExpand++;
                            vSizeAndPointerToNode.push_back(make_pair(n3.vKeys.size(),&lNodes.front()));
                            lNodes.front().lit = lNodes.begin();
                        }
                    }
                    if(n4.vKeys.size()>0)
                    {
                        lNodes.push_front(n4);
                        if(n4.vKeys.size()>1)
                        {
                            nToExpand++;
                            vSizeAndPointerToNode.push_back(make_pair(n4.vKeys.size(),&lNodes.front()));
                            lNodes.front().lit = lNodes.begin();
                        }
                    }

                    lit=lNodes.erase(lit);
                    continue;
                }
            }

            // Finish if there are more nodes than required features
            // or all nodes contain just one point
            if((int)lNodes.size()>=N || (int)lNodes.size()==prevSize)
            {
                bFinish = true;
            }
            else if(((int)lNodes.size()+nToExpand*3)>N)
            {

                while(!bFinish)
                {

                    prevSize = lNodes.size();

                    vector<pair<int,ExtractorNodeHG*> > vPrevSizeAndPointerToNode = vSizeAndPointerToNode;
                    vSizeAndPointerToNode.clear();

                    sort(vPrevSizeAndPointerToNode.begin(),vPrevSizeAndPointerToNode.end());
                    for(int j=vPrevSizeAndPointerToNode.size()-1;j>=0;j--)
                    {
                        ExtractorNodeHG n1,n2,n3,n4;
                        vPrevSizeAndPointerToNode[j].second->DivideNode(n1,n2,n3,n4);

                        // Add childs if they contain points
                        if(n1.vKeys.size()>0)
                        {
                            lNodes.push_front(n1);
                            if(n1.vKeys.size()>1)
                            {
                                vSizeAndPointerToNode.push_back(make_pair(n1.vKeys.size(),&lNodes.front()));
                                lNodes.front().lit = lNodes.begin();
                            }
                        }
                        if(n2.vKeys.size()>0)
                        {
                            lNodes.push_front(n2);
                            if(n2.vKeys.size()>1)
                            {
                                vSizeAndPointerToNode.push_back(make_pair(n2.vKeys.size(),&lNodes.front()));
                                lNodes.front().lit = lNodes.begin();
                            }
                        }
                        if(n3.vKeys.size()>0)
                        {
                            lNodes.push_front(n3);
                            if(n3.vKeys.size()>1)
                            {
                                vSizeAndPointerToNode.push_back(make_pair(n3.vKeys.size(),&lNodes.front()));
                                lNodes.front().lit = lNodes.begin();
                            }
                        }
                        if(n4.vKeys.size()>0)
                        {
                            lNodes.push_front(n4);
                            if(n4.vKeys.size()>1)
                            {
                                vSizeAndPointerToNode.push_back(make_pair(n4.vKeys.size(),&lNodes.front()));
                                lNodes.front().lit = lNodes.begin();
                            }
                        }

                        lNodes.erase(vPrevSizeAndPointerToNode[j].second->lit);

                        if((int)lNodes.size()>=N)
                            break;
                    }

                    if((int)lNodes.size()>=N || (int)lNodes.size()==prevSize)
                        bFinish = true;

                }
            }
        }

        // Retain the best point in each node
        list<HighGradientPoint*> vResultKeys;

        for(list<ExtractorNodeHG>::iterator lit=lNodes.begin(); lit!=lNodes.end(); lit++)
        {
            vector<HighGradientPoint*> &vNodeKeys = lit->vKeys;
            HighGradientPoint* pKP = vNodeKeys[0];

            // TODO! Now prefer older features
            float age = pKP->refKF->mnId;

            for(size_t k=1;k<vNodeKeys.size();k++)
            {
                if(vNodeKeys[k]->refKF->mnId > age)
                {
                    pKP = vNodeKeys[k];
                    age = pKP->refKF->mnId;
                }
            }

            vResultKeys.push_back(pKP);
        }

        // Joining features after octree and features outside the image
        vResultKeys.splice(vResultKeys.end(), outsideCurImage);


        return vResultKeys;
    }

}
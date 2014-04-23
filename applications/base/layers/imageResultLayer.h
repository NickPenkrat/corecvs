#pragma once
/**
 * \file imageResultLayer.h
 *
 * \date Oct 17, 2012
 **/
#include <QtGui/QImage>
#include <QtGui/QPainter>

#include "global.h"

#include "resultImage.h"
#include "outputStyle.h"
#include "frames.h"
#include "g12Image.h"
#include "painterHelpers.h"



class ImageResultLayer : public ResultLayerBase
{

    OutputStyle::OutputStyle mStyle;
    QImage* mImages[Frames::MAX_INPUTS_NUMBER];
    bool mShowLeftFrame;

public:

template<class BufferType>
    ImageResultLayer(
        OutputStyle::OutputStyle style,
        BufferType* images[Frames::MAX_INPUTS_NUMBER],
        bool showLeftFrame = false
    );

/*    ImageResultLayer(
        OutputStyle::OutputStyle style,
        RGB24Buffer* images[Frames::MAX_INPUTS_NUMBER],
        bool showLeftFrame = false
    );*/

    ImageResultLayer(
        G12Buffer* image
    );

    ImageResultLayer(
        RGB24Buffer* image
    );


    /** Polymorphic type so virtual destructor needed */
    virtual ~ImageResultLayer();

    virtual void drawImage (QImage *image);

    virtual int  modifyHeight (int height)
    {
    	return height;
    }

    virtual int  modifyWidth (int width)
    {
        if (mStyle == OutputStyle::SIDEBYSIDE_STEREO) {
            return 2 * width;
        } else {
            return width;
        }
    }

    QImage* getImage(const int id) const {
        return mImages[id];
    }

    virtual void print() const {
        for (int i = 0; i < Frames::MAX_INPUTS_NUMBER; i++)
        {
            printf("Image %d: ", i);
            if (mImages[i] == NULL) {
                printf(" NULL\n");
                continue;
            }
            printf("[%dx%d]\n", mImages[i]->height(), mImages[i]->width());
        }
    }
};


/* EOF */

#ifndef CLASSES_H_INCLUDED
#define CLASSES_H_INCLUDED

class anchor_object
{
public:

OBJ_typ typ;

anchor_object()
{
typ=OBJ_typ_terrain;
}

anchor_object(OBJ_typ obj)
{
typ = obj;
}

virtual void setRotation(irr::core::vector3df rot) = 0;

virtual void setPosition(irr::core::vector3df pos) = 0;


};

#endif // CLASSES_H_INCLUDED

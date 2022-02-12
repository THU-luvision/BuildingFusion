#ifndef LABEL_PROCESSING_H
#define LABEL_PROCESSING_H

static int get_semantic_label(long label)
{
    
    return label / 10000 + (label%10000 == 9999); 
}
static int get_instance_label(long label)
{
    int result = label % 10000;
    if(result == 9999)
    result = -1;
    return result;
}

#endif
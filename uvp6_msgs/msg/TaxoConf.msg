Header header
string frame_name # Frame name
string configuration_name # Name used to set this configuration into the field Taxo_conf
string model_reference # Taxonomic classification model to be use
int32 max_size_for_classification # Maximum vignette size to perform the embedded classification
int32 model_nb_classes # Number of classes used by the classification model
int32[] taxo_ids # Ecotaxa taxonomic unique identifier for each class array (1-40)

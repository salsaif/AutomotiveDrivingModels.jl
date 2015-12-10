using AutomotiveDrivingModels

include(Pkg.dir("AutomotiveDrivingModels", "src", "io", "filesystem_utils.jl"))

##############################
# PARAMETERS
##############################

const INCLUDE_FILE = Pkg.dir("AutomotiveDrivingModels", "scripts", "extract_params.jl")
include(INCLUDE_FILE)

type DataSubset
    name::AbstractString
    behavior_to_match::UInt16
    behavior_to_avoid::UInt16
end

subsets = [DataSubset("freeflow", ContextClass.FREEFLOW, ContextClass.NULL),
           DataSubset("following", ContextClass.FOLLOWING, ContextClass.NULL),
           DataSubset("lanechange", ContextClass.LANECHANGE, ContextClass.NULL)]

##############################
# LOAD
##############################

const SAVE_DIR = joinpath(EVALUATION_DIR, "bincounts_opt")
if !isdir(SAVE_DIR)
    mkdir(SAVE_DIR)
end

features = collect(FeaturesNew.allfeatures())
# filters = [FeaturesNew.Feature_IsClean{symbol(TARGET_SET.lat)}(),
#                           Feature_IsClean{symbol(TARGET_SET.lon)}()]

runlog_extract_params = RunLogSegmentExtractParameters(SIM_HORIZON_IN_FRAMES, SIM_HISTORY_IN_FRAMES,
                                                       FRAMESKIP_BETWEEN_EXTRACTED_SCENES, PDSET_FRAMES_PER_SIM_FRAME)
dataset_extract_params = DatasetExtractParams(0x0000, 0x0000, features, runlog_extract_params)
runlog_filepaths = filter!(s->splitext(s)[2] == ".jld", readdir(RUNLOG_DIR))
for i in 1 : length(runlog_filepaths)
    runlog_filepaths[i] = joinpath(RUNLOG_DIR, runlog_filepaths[i])
end

for subset in subsets

    dataset_extract_params.behavior_to_match = subset.behavior_to_match
    dataset_extract_params.behavior_to_avoid = subset.behavior_to_avoid

    tic()
    model_training_data = pull_model_training_data(dataset_extract_params, runlog_filepaths)
    toc()

    dataset_filepath = joinpath(EVALUATION_DIR, "dataset2_" * subset.name * ".jld")
    JLD.save(dataset_filepath, "model_training_data", model_training_data,
                               "extract_params", dataset_extract_params)

    println("num pdset_segments: ", length(model_training_data.runlog_segments))
    println("size of dataframe:  ", size(model_training_data.dataframe))

end
println("[DONE]")

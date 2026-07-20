clear;
clc;

simDir = fileparts(mfilename('fullpath'));

caseFiles = {
    'case1_A_sine_B_sine.csv', 'sine',     'sine';
    'case2_A_triangle_B_triangle.csv', 'triangle', 'triangle';
    'case3_A_triangle_B_sine.csv', 'triangle', 'sine';
    'case4_A_sine_B_triangle.csv', 'sine',     'triangle'
};

summaryCase = {};
summaryTrueA = {};
summaryTrueB = {};
summaryTotal = [];
summaryCorrect = [];
summaryAccuracy = [];

wrongCase = {};
wrongFA = [];
wrongFB = [];
wrongTrueA = {};
wrongTrueB = {};
wrongPredA = {};
wrongPredB = {};
wrongBranch = {};
wrongA3Ratio = [];
wrongA5Ratio = [];
wrongB3Ratio = [];
wrongB5Ratio = [];

for caseIdx = 1:size(caseFiles, 1)
    csvPath = fullfile(simDir, caseFiles{caseIdx, 1});
    trueA = caseFiles{caseIdx, 2};
    trueB = caseFiles{caseIdx, 3};

    data = readtable(csvPath);
    totalCount = height(data);
    correctCount = 0;

    for row = 1:totalCount
        fA = data.fA(row);
        fB = data.fB(row);

        [predA, predB, detail] = classifyByMdAlgorithm(fA, fB, trueA, trueB);
        isCorrect = strcmp(predA, trueA) && strcmp(predB, trueB);

        if isCorrect
            correctCount = correctCount + 1;
        else
            wrongCase{end + 1, 1} = caseFiles{caseIdx, 1};
            wrongFA(end + 1, 1) = fA;
            wrongFB(end + 1, 1) = fB;
            wrongTrueA{end + 1, 1} = trueA;
            wrongTrueB{end + 1, 1} = trueB;
            wrongPredA{end + 1, 1} = predA;
            wrongPredB{end + 1, 1} = predB;
            wrongBranch{end + 1, 1} = detail.branch;
            wrongA3Ratio(end + 1, 1) = detail.a3Ratio;
            wrongA5Ratio(end + 1, 1) = detail.a5Ratio;
            wrongB3Ratio(end + 1, 1) = detail.b3Ratio;
            wrongB5Ratio(end + 1, 1) = detail.b5Ratio;
        end
    end

    summaryCase{end + 1, 1} = caseFiles{caseIdx, 1};
    summaryTrueA{end + 1, 1} = trueA;
    summaryTrueB{end + 1, 1} = trueB;
    summaryTotal(end + 1, 1) = totalCount;
    summaryCorrect(end + 1, 1) = correctCount;
    summaryAccuracy(end + 1, 1) = correctCount / totalCount;
end

summary = table(summaryCase, summaryTrueA, summaryTrueB, summaryTotal, ...
    summaryCorrect, summaryAccuracy, ...
    'VariableNames', {'caseFile', 'trueA', 'trueB', 'total', 'correct', 'accuracy'});

wrongCases = table(wrongCase, wrongFA, wrongFB, wrongTrueA, wrongTrueB, ...
    wrongPredA, wrongPredB, wrongBranch, wrongA3Ratio, wrongA5Ratio, ...
    wrongB3Ratio, wrongB5Ratio, ...
    'VariableNames', {'caseFile', 'fA', 'fB', 'trueA', 'trueB', ...
    'predA', 'predB', 'branch', 'a3Ratio', 'a5Ratio', 'b3Ratio', 'b5Ratio'});

writetable(summary, fullfile(simDir, 'simulation_summary.csv'));
writetable(wrongCases, fullfile(simDir, 'wrong_cases.csv'));

disp(summary);

if isempty(wrongCase)
    fprintf('No wrong cases found.\n');
else
    disp(wrongCases);
end

function [predA, predB, detail] = classifyByMdAlgorithm(fA, fB, trueA, trueB)
    TH3 = 0.08;
    TH5 = 0.02;

    baseA = spectralAmpAt(fA, fA, trueA, fB, trueB);
    baseB = spectralAmpAt(fB, fA, trueA, fB, trueB);

    a3Ratio = ratioAt(3 * fA, baseA, fA, trueA, fB, trueB);
    a5Ratio = ratioAt(5 * fA, baseA, fA, trueA, fB, trueB);
    b3Ratio = ratioAt(3 * fB, baseB, fA, trueA, fB, trueB);
    b5Ratio = ratioAt(5 * fB, baseB, fA, trueA, fB, trueB);

    if freqEqual(3 * fA, fB)
        predA = classifyByRatio(a5Ratio, TH5);
        predB = classifyByRatio(b3Ratio, TH3);
        branch = '3fA=fB: use 5fA for A, use 3fB for B';
    elseif a3Ratio > TH3
        predA = 'triangle';

        if freqEqual(5 * fA, 3 * fB)
            predB = classifyByRatio(b5Ratio, TH5);
            branch = '3fA~=fB and A3 exists, 5fA=3fB: use 5fB for B';
        else
            predB = classifyByRatio(b3Ratio, TH3);
            branch = '3fA~=fB and A3 exists, 5fA~=3fB: use 3fB for B';
        end
    else
        predA = 'sine';
        predB = classifyByRatio(b3Ratio, TH3);

        if a5Ratio > TH5
            branch = 'A3~=0 false but A5 has energy: A=sine, use 3fB for B';
        else
            branch = 'A3~=0 false: A=sine, use 3fB for B';
        end
    end

    detail = struct( ...
        'branch', branch, ...
        'a3Ratio', a3Ratio, ...
        'a5Ratio', a5Ratio, ...
        'b3Ratio', b3Ratio, ...
        'b5Ratio', b5Ratio);
end

function waveType = classifyByRatio(ratio, threshold)
    if ratio > threshold
        waveType = 'triangle';
    else
        waveType = 'sine';
    end
end

function ratio = ratioAt(freq, baseAmp, fA, typeA, fB, typeB)
    if baseAmp <= 1.0e-12
        ratio = 0.0;
    else
        ratio = spectralAmpAt(freq, fA, typeA, fB, typeB) / baseAmp;
    end
end

function amp = spectralAmpAt(freq, fA, typeA, fB, typeB)
    amp = singleSignalAmpAt(freq, fA, typeA) + singleSignalAmpAt(freq, fB, typeB);
end

function amp = singleSignalAmpAt(freq, baseFreq, waveType)
    SINE_BASE_AMP = 1.0;
    TRIANGLE_BASE_AMP = 0.811;

    amp = 0.0;

    if freqEqual(freq, baseFreq)
        if strcmp(waveType, 'triangle')
            amp = TRIANGLE_BASE_AMP;
        else
            amp = SINE_BASE_AMP;
        end
    elseif strcmp(waveType, 'triangle') && freqEqual(freq, 3 * baseFreq)
        amp = TRIANGLE_BASE_AMP / 9.0;
    elseif strcmp(waveType, 'triangle') && freqEqual(freq, 5 * baseFreq)
        amp = TRIANGLE_BASE_AMP / 25.0;
    end
end

function tf = freqEqual(leftFreq, rightFreq)
    tf = abs(leftFreq - rightFreq) < 1.0e-6;
end

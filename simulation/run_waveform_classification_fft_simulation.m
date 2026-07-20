clear;
clc;

simDir = fileparts(mfilename('fullpath'));

FFT_LEN = 1024;
FS = 1025641.0;
PEAK_GUARD = 5;
HARMONIC_SEARCH_HALF_WIDTH = 2;
TRI_3RD_RATIO_THRESHOLD = 0.08;
TRI_5TH_RATIO_THRESHOLD = 0.02;

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
wrongABin = [];
wrongBBin = [];
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

        x = makeMixedSignal(fA, trueA, fB, trueB, FS, FFT_LEN);
        mag = fftMagLikeStm32(x, FFT_LEN);
        [predA, predB, detail] = classifyFromFftMag(mag, FS, FFT_LEN, ...
            PEAK_GUARD, HARMONIC_SEARCH_HALF_WIDTH, ...
            TRI_3RD_RATIO_THRESHOLD, TRI_5TH_RATIO_THRESHOLD);

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
            wrongABin(end + 1, 1) = detail.aBin;
            wrongBBin(end + 1, 1) = detail.bBin;
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
    wrongPredA, wrongPredB, wrongABin, wrongBBin, wrongBranch, ...
    wrongA3Ratio, wrongA5Ratio, wrongB3Ratio, wrongB5Ratio, ...
    'VariableNames', {'caseFile', 'fA', 'fB', 'trueA', 'trueB', ...
    'predA', 'predB', 'aBin', 'bBin', 'branch', ...
    'a3Ratio', 'a5Ratio', 'b3Ratio', 'b5Ratio'});

writetable(summary, fullfile(simDir, 'fft_simulation_summary.csv'));
writetable(wrongCases, fullfile(simDir, 'fft_wrong_cases.csv'));

disp(summary);

if isempty(wrongCase)
    fprintf('No wrong cases found.\n');
else
    disp(wrongCases);
end

function x = makeMixedSignal(fA, typeA, fB, typeB, fs, n)
    t = (0:n - 1) / fs;
    xA = makeWave(fA, typeA, t);
    xB = makeWave(fB, typeB, t);
    x = xA + xB;
end

function x = makeWave(freq, waveType, t)
    if strcmp(waveType, 'triangle')
        phase = freq * t;
        x = 2.0 * abs(2.0 * (phase - floor(phase + 0.5))) - 1.0;
        x = 0.5 * x;
    else
        x = 0.5 * sin(2.0 * pi * freq * t);
    end
end

function mag = fftMagLikeStm32(x, n)
    windowCorrection = 1.55;
    dc = mean(x);
    win = 0.5 * (1.0 - cos(2.0 * pi * (0:n - 1) / (n - 1)));
    y = (x - dc) .* win;
    mag = abs(fft(y, n));
    mag(1) = mag(1) / n * windowCorrection;
    mag(2:end) = mag(2:end) * 2.0 / n * windowCorrection;
end

function [predA, predB, detail] = classifyFromFftMag(mag, fs, n, ...
    peakGuard, harmonicHalfWidth, th3, th5)

    fftHalfBin = n / 2;
    fftLastBin = fftHalfBin - 1;

    [index1, index2] = findTop2Peaks(mag, fftHalfBin, peakGuard);

    if index1 == 0 || index2 == 0
        predA = 'sine';
        predB = 'sine';
        detail = makeDetail(0, 0, 0, 0, 0, 0, 'top-2 peak search failed');
        return;
    end

    aBin = min(index1, index2);
    bBin = max(index1, index2);

    a3Bin = aBin * 3;
    a5Bin = aBin * 5;
    b3Bin = bBin * 3;
    b5Bin = bBin * 5;

    a3Ratio = harmonicRatioNear(mag, aBin, a3Bin, harmonicHalfWidth, fftLastBin);
    a5Ratio = harmonicRatioNear(mag, aBin, a5Bin, harmonicHalfWidth, fftLastBin);
    b3Ratio = harmonicRatioNear(mag, bBin, b3Bin, harmonicHalfWidth, fftLastBin);
    b5Ratio = harmonicRatioNear(mag, bBin, b5Bin, harmonicHalfWidth, fftLastBin);

    if binsNear(a3Bin, bBin, harmonicHalfWidth)
        predA = classifyByRatio(a5Ratio, th5);
        predB = classifyByRatio(b3Ratio, th3);
        branch = '3fA=fB: use 5fA for A, use 3fB for B';
    elseif a3Ratio > th3
        predA = 'triangle';

        if binsNear(a5Bin, b3Bin, harmonicHalfWidth)
            predB = classifyByRatio(b5Ratio, th5);
            branch = '3fA~=fB and A3 exists, 5fA=3fB: use 5fB for B';
        else
            predB = classifyByRatio(b3Ratio, th3);
            branch = '3fA~=fB and A3 exists, 5fA~=3fB: use 3fB for B';
        end
    else
        predA = 'sine';
        predB = classifyByRatio(b3Ratio, th3);

        if a5Ratio > th5
            branch = 'A3~=0 false but A5 has energy: A=sine, use 3fB for B';
        else
            branch = 'A3~=0 false: A=sine, use 3fB for B';
        end
    end

    detail = makeDetail(aBin, bBin, a3Ratio, a5Ratio, b3Ratio, b5Ratio, branch);

    %#ok<NASGU>
    fs = fs;
end

function [index1, index2] = findTop2Peaks(mag, fftHalfBin, peakGuard)
    max1 = 0.0;
    max2 = 0.0;
    index1 = 0;
    index2 = 0;

    for bin = 2:fftHalfBin - 2
        if getBinMag(mag, bin) > getBinMag(mag, bin - 1) && ...
                getBinMag(mag, bin) > getBinMag(mag, bin + 1)
            if getBinMag(mag, bin) > max1
                max1 = getBinMag(mag, bin);
                index1 = bin;
            end
        end
    end

    for bin = 2:fftHalfBin - 2
        if binsNear(bin, index1, peakGuard)
            continue;
        end

        if getBinMag(mag, bin) > getBinMag(mag, bin - 1) && ...
                getBinMag(mag, bin) > getBinMag(mag, bin + 1)
            if getBinMag(mag, bin) > max2
                max2 = getBinMag(mag, bin);
                index2 = bin;
            end
        end
    end
end

function ratio = harmonicRatioNear(mag, baseBin, harmonicBin, halfWidth, lastBin)
    if baseBin > lastBin || harmonicBin > lastBin
        ratio = 0.0;
        return;
    end

    startBin = max(harmonicBin - halfWidth, 2);
    endBin = min(harmonicBin + halfWidth, lastBin);
    harmonicMag = max(getBinMag(mag, startBin:endBin));
    baseMag = getBinMag(mag, baseBin);

    if baseMag <= 1.0e-12
        ratio = 0.0;
    else
        ratio = harmonicMag / baseMag;
    end
end

function value = getBinMag(mag, bin)
    value = mag(bin + 1);
end

function waveType = classifyByRatio(ratio, threshold)
    if ratio > threshold
        waveType = 'triangle';
    else
        waveType = 'sine';
    end
end

function tf = binsNear(leftBin, rightBin, tolerance)
    tf = abs(leftBin - rightBin) <= tolerance;
end

function detail = makeDetail(aBin, bBin, a3Ratio, a5Ratio, b3Ratio, b5Ratio, branch)
    detail = struct( ...
        'aBin', aBin, ...
        'bBin', bBin, ...
        'a3Ratio', a3Ratio, ...
        'a5Ratio', a5Ratio, ...
        'b3Ratio', b3Ratio, ...
        'b5Ratio', b5Ratio, ...
        'branch', branch);
end

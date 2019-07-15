
class Recognizer(object):
    def __init__(self):
        self.desc = "Recognizer"

    @staticmethod
    def any(recognizers):
        return RecognizerAnyWrapper(recognizers)

    @staticmethod
    def all(recognizers):
        return RecognizerAllWrapper(recognizers)

    def stop(self):
        pass

    def start(self):
        pass

    def detect(self, pose, goal):
        return False

class RecognizerAnyWrapper(Recognizer):
    def __init__(self, recognizers):
        super(RecognizerAnyWrapper, self).__init__()
        self._recognizers = recognizers
        self.desc = "({})".format(" or ".join([r.desc for r in recognizers]))

    def stop(self):
        for r in self._recognizers:
            r.stop()

    def start(self):
        for r in self._recognizers:
            r.start()

    def detect(self, pose, goal):
        results, reasons = zip(*[r.detect(pose, goal) for r in self._recognizers])
        result_any = any(results)
        reasons = ["{}".format(reason) for result, reason in zip(results, reasons) if result_any == result]
        return result_any, ", ".join(reasons)

class RecognizerAllWrapper(Recognizer):
    def __init__(self, recognizers):
        super(RecognizerAllWrapper, self).__init__()
        self._recognizers = recognizers
        self.desc = "({})".format(" and ".join([r.desc for r in recognizers]))

    def stop(self):
        for r in self._recognizers:
            r.stop()

    def start(self):
        for r in self._recognizers:
            r.start()

    def detect(self, pose, goal):
        results, reasons = zip(*[r.detect(pose, goal) for r in self._recognizers])
        result_all = all(results)
        reasons = ["{}".format(reason) for result, reason in zip(results, reasons) if result_all == result]
        return result_all, ", ".join(reasons)

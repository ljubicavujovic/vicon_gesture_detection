import nltk
import numpy as np
from nltk.corpus import wordnet
import pickle as pickle
from object import Object
from scipy.stats import expon
from nltk.tokenize import word_tokenize

"""
Class for procesing text generated from speech.
""""
class Speech:
    def __init__(self):
        #depicle list of objects
        with open("objects.pickle", "rb") as f:
            objects = pickle.load(f)
        self.objects = objects
        self.epsilon = len(self.objects)

        # define grammar for sentances
        self.grammar = nltk.CFG.fromstring("""
          S -> NP VP
          VP -> V NP | V NP PP
          PP -> P NP
          V -> "pick" | "put"
          NP -> Det N | Det N PP | Det Adj N | Adj N | Det N Adj | N Adj | Det Adj N N | N
          Det -> "a" | "an" | "the" | "my" | "this"
          Adj -> "first" | "second" | "yellow" | "blue" | "red" | "one" | "two"
          Conj -> "and"
          N -> "cube" | "plate" | "banana" | "coffee" | "Fetch" | "object" | "master" | "chef"
          P -> "behind" | "near" | "left to" | "right to" | "far" | "from"
          """)
        # define tags which will be used for creating synset from word.
        self.tags = {"NOUN": ".n.01", "VERB": ".v.01", "ADP": ".r.01", "ADJ": ".n.01", "NUM": ".n.01", "ADV": ".n.01"}
        self.coordinates = np.zeros((self.epsilon, 3))

    def remove_det(self, sent):
        """
        Removes determiners from sentance

        Arguments:
        sent -- list of words in sentance

        Returns:
        sent -- list of words in sentance without determiners
        """
        taged = nltk.pos_tag(sent)
        for tag in taged:
            if tag[1] == "DT":
                sent.remove(tag[0])
        return sent

    def joint_score(self, object, word):
        """
        Calculates joint score between object and word. Joint score represents
        context similarity between words used to describe an object contained in
        vocabulary and particular word.

        Arguments:
        object -- object of interest
        word -- string to be compared

        Returns:
        score -- sum of WUP similarities between word and vocabulary
                 words normalizes by number of words in vocabulary
        """
        sum_score = 0
        w1 = wordnet.synset(word + self.tags[nltk.pos_tag(word.split(), "universal")[0][1]])
        for w in object.vocabulary:
            w2 = wordnet.synset(w + self.tags[nltk.pos_tag(w.split(), "universal")[0][1]])
            sum_score += w1.wup_similarity(w2)

        return sum_score/len(object.vocabulary)

    def joint_score_sum(self, word):
        """
        Calculates sum of joint scores for particular word and all objects

        Arguments:
        word -- string to be compared

        Returns:
        sum_score -- sum of all joint scores
        """
        sum_score = 0
        for i in range(0, self.epsilon):
            wi = self.joint_score(self.objects[i], word)
            sum_score += wi
        return sum_score

    def get_coordinates(self, object):
        """
        Gets coordinates of object

        Arguments:
        object -- object of interest

        Returns:
        coordinates -- 3D coordinates
        """
        index = self.objects.index(object)
        print self.coordinates
        return self.coordinates[index, :]

    def near(self, target, reference):
        """
        Returns norm of expenential probabilty density function assosiated with
        preposition near. Input to PDF is negative absolute value of coordinate
        difference between reference and target object

        Arguments:
        target -- target object
        reference -- object references by preposition

        Returns:
        probability
        """
        target_coordinates = self.get_coordinates(target)
        reference_coordinates = self.get_coordinates(reference)
        print target, np.linalg.norm(np.exp(-abs(target_coordinates - reference_coordinates)))
        if target.name == reference.name:
            return 0
        else:
            return np.linalg.norm(np.exp(-abs(target_coordinates - reference_coordinates)))

    def far(self, target, reference):
        """
        Returns norm of expenential probabilty density function assosiated with
        preposition far. Input to PDF is positive absolute value of coordinate
        difference between reference and target object

        Arguments:
        target -- target object
        reference -- object references by preposition

        Returns:
        probability
        """
        target_coordinates = self.get_coordinates(target)
        reference_coordinates = self.get_coordinates(reference)
        print target, np.linalg.norm(np.exp(abs(target_coordinates - reference_coordinates)))
        return np.linalg.norm(np.exp(abs(target_coordinates - reference_coordinates)))

    def preposition_locator(self, preposition, reference):
        """
        Assosiates PDF to preposition and calculates distribution for all
        objects for reference object from input

        Argument:
        preposition -- string
        reference -- object which is references by preposition

        Returns:
        distribution for all objects
        """
        function = {"near": self.near,
                    "far": self.far
                    }
        p = np.zeros((self.epsilon, 1))
        for i in range(0, self.epsilon):
            p[i] = function[preposition](self.objects[i], reference)
        p = p/sum(p)
        return p

    def get_constituents(self, sentance, type):
        """
        Parses the sentance based on specified grammar and returns particular
        constituent specified in type.

        Arguments:
        sentance -- string
        type -- string which represents type of constituent. Possible values NP,
                VP, PP, V, N, A ...

        Returns:
        p -- list of words in particular constituent
        """
        rd_parser = nltk.RecursiveDescentParser(self.grammar)
        words = sentance.split()
        for tree in rd_parser.parse(words):
            p = [" ".join(i.leaves()) for i in tree.subtrees() if i.label() == type]
        return p

    def get_grounding_probability(self, grounding, object, alpha=0.005):
        """
        Calculates probability that we are talking about object given the grounding.

        Arguments:
        grounding -- list of words in phrase
        object -- object for which probability is Calculated
        alpha -- smoothening parametar

        Returns:
        p -- probability
        """
        grounding = self.remove_det(grounding.split())
        p = 1
        for i in range(0, len(grounding)):
            p *= (self.joint_score(object, grounding[i]) + alpha)/(self.joint_score_sum(grounding[i]) + alpha*self.epsilon)
        return p

    def get_grounding_distribution(self, grounding):
        """
        Calculates the distribution for all objects given the grounding

        Arguments:
        grounding -- list of words in phrase

        Returns:
        distribution
        """
        grounding_distribution = np.zeros((self.epsilon, 1))
        for i in range(0, self.epsilon):
            grounding_distribution[i] = self.get_grounding_probability(grounding, self.objects[i])
        return grounding_distribution/sum(grounding_distribution)

    def get_preposition_distribution(self, preposition_phrase):
        """
        Determines what is reference object in preposition phrase by calculating
        grounding distribution for words in phrase and taking one with Maximum
        probability. Calculates distribution over all objects given the reference
        object and particular preposition

        Arguments:
        preposition_phrase -- lis of words in PP

        Returns:
        distribution
        """
        taged = nltk.pos_tag(preposition_phrase.split(), "universal")
        preposition = taged[0][0]
        reference = taged[1][0]

        grounding_distribution = self.get_grounding_distribution(reference)
        argmax = int(np.argmax(grounding_distribution))

        preposition_distribution = grounding_distribution[argmax]*self.preposition_locator(preposition, self.objects[argmax])

        return preposition_distribution/sum(preposition_distribution)

    def observation_distribution(self, sentance):
        """
        Generates observation from sentance. First, identifies if sentance has
        NP or PP, then based on the content calculates distribution

        Arguments:
        sentance -- string

        Returns:
        distribution
        """
        phrases = ["NP", "V", "PP"]
        d = {}
        for phrase in phrases:
            d[phrase] = self.get_constituents(sentance, phrase)
        if not d["PP"]:
            print d["NP"]
            distribution = self.get_grounding_distribution(d["NP"][1])
        else:
            for p in d["PP"]:
                distribution = self.get_preposition_distribution(p)

        return distribution

S = Speech()
coordinates = np.zeros((5, 3))
coordinates.fill(5)
coordinates[4] = np.array([2, 2, 2])
coordinates[3] = np.array([2, 2, 3])

S.coordinates = coordinates
print "GD", S.observation_distribution("Fetch pick this red plate")

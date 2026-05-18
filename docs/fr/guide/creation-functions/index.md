---
description: "Explique systématiquement les fonctions de Creation Function de RxJS (fonctions de création d'Observable), leurs différences avec Pipeable Operator, l'utilisation de base et les sept catégories (Basic Creation, Loop Generation, HTTP Communication, Joining, Selection and Splitting, Conditional Splitting and Control). Les caractéristiques de chaque fonction et un guide de sélection pour différentes utilisations sont fournis."
---

# Creation Function

Dans RxJS, il existe deux formes différentes de **Fonctions de Création** pour créer des Observable et de **Pipeable Operators** pour convertir des Observable existants.

Cette page décrit les concepts de base des Creation Function et les sept catégories principales.

## Qu'est-ce que les Creation Function ?

**Les fonctions de création sont des fonctions permettant de créer de nouveaux Observable.

```typescript
import { of, from, interval } from 'rxjs';

// Creation FunctionUtilisé comme
const obs1$ = of(1, 2, 3);
const obs2$ = from([4, 5, 6]);
const obs3$ = interval(1000);
```

Elles sont importées directement du paquetage `rxjs` et appelées en tant que fonctions pour créer un Observable.

## Différences avec Pipeable Operator

Les Creation Function et les Pipeable Operator ont des utilisations et des usages différents. Voir le tableau ci-dessous pour voir les différences entre eux.

| Caractéristiques | Creation Function | Pipeable Operator |
|---|---|---|
| **Utilisation**. | Création d'un nouvel Observable | Conversion d'un Observable existant |
| **Importer**. | `rxjs`. | `rxjs/operators`. |
| **Comment l'utiliser ? | Appeler directement en tant que fonction | Utilisation dans `.pipe()`. |
| **Exemple**. | `concat(obs1$, obs2$)` | `obs1$.pipe(concatWith(obs2$))` |

### Exemple de Creation Function

La Creation Function est utilisée pour combiner directement plusieurs Observable.

```typescript
import { concat, of } from 'rxjs';

const obs1$ = of(1, 2, 3);
const obs2$ = of(4, 5, 6);

// Creation Function Utilisé comme
concat(obs1$, obs2$).subscribe(console.log);
// Sortie de l'usine: 1, 2, 3, 4, 5, 6
```

### Exemple de Pipeable Operator.

Le Pipeable Operator est utilisé pour ajouter un processus de conversion à un Observable existant.

```typescript
import { of } from 'rxjs';
import { concatWith } from 'rxjs';

const obs1$ = of(1, 2, 3);
const obs2$ = of(4, 5, 6);

// Pipeable Operator Utilisé comme
obs1$.pipe(
  concatWith(obs2$)
).subscribe(console.log);
// Sortie de l'usine: 1, 2, 3, 4, 5, 6
```

## Critères d'utilisation.

Le choix entre Creation Function et Pipeable Operator est déterminé par les critères suivants

### Quand la Creation Function doit être utilisée.

La Creation Function convient lorsque vous souhaitez exploiter plusieurs Observable au même niveau ou lorsque vous souhaitez créer un Observable à partir de zéro.

- Lorsque vous combinez plusieurs Observable au même niveau**.

```typescript
  concat(obs1$, obs2$, obs3$)
  merge(click$, hover$, scroll$)
  ```

- **Depuis le débutObservableQuand créer**
  ```typescript
  of(1, 2, 3)
  from([1, 2, 3])
  interval(1000)
  ```

### Pipeable Operator Si vous devez utiliser

Pipeable Operatorest approprié lorsque vous souhaitez ajouter un processus à un processus existant ou lorsque vous souhaitez enchaîner plusieurs opérations.Observableexistant ou lorsque vous souhaitez enchaîner plusieurs opérations.

- **Si vous souhaitez ajouter un processus à un processus existantObservableLorsque vous souhaitez ajouter un processus à un processus existant**
  ```typescript
  obs1$.pipe(
    map(x => x * 2),
    concatWith(obs2$),
    filter(x => x > 5)
  )
  ```

- **Lorsque vous enchaînez plusieurs opérations en tant que pipeline**

## Creation Functions Catégories de

Dans ce chapitre,Creation Functionsaux7Vous apprendrez à les diviser en trois catégories.

### Liste de toutes les catégories

Le tableau ci-dessous présente toutes les catégories et les fonctions qu'elles contiennent. Cliquez sur le nom de chaque fonction pour accéder à la page détaillée.

| Catégorie | Description de la fonction | Fonctions principales | Cas d'utilisation typiques |
|---------|------|-----------|-------------------|
| **[Système de création de base](/fr/guide/creation-functions/basic/)** | Fonctions les plus élémentaires et les plus fréquemment utilisées. Données, tableaux, événements et tempsObservableCréation de | [of](/fr/guide/creation-functions/basic/of), [from](/fr/guide/creation-functions/basic/from), [fromEvent](/fr/guide/creation-functions/basic/fromEvent), [interval](/fr/guide/creation-functions/basic/interval), [timer](/fr/guide/creation-functions/basic/timer) | Test de valeur fixe, flux de données existantes,DOMle traitement des événements, l'interrogation et l'exécution différée. |
| **[Systèmes de génération de boucles](/fr/guide/creation-functions/loop/)** | forSystèmes de génération de boucles tels que les instructions et le traitement des boucles.whileTraitement de boucle tel queObservableExpression sous la forme d'un | [range](/fr/guide/creation-functions/loop/range), [generate](/fr/guide/creation-functions/loop/generate) | Génération de nombres séquentiels, traitement par lots, transitions d'état complexes, calculs mathématiques |
| **[HTTPSystèmes de communication](/fr/guide/creation-functions/http-communication/)** | HTTPLa communication est traitée commeObservableTraitée comme | [ajax](/fr/guide/creation-functions/http-communication/ajax), [fromFetch](/fr/guide/creation-functions/http-communication/fromFetch) | XMLHttpRequestbasée surHTTPla communication,Fetch APIbasée surHTTPla communication,REST APIAppel |
| **[Système couplé](/fr/guide/creation-functions/combination/)** | Un système dans lequel plusieursObservableaux1sont liés à un seul. Le moment et l'ordre de publication dépendent de la méthode de couplage. | [concat](/fr/guide/creation-functions/combination/concat), [merge](/fr/guide/creation-functions/combination/merge), [combineLatest](/fr/guide/creation-functions/combination/combineLatest), [zip](/fr/guide/creation-functions/combination/zip), [forkJoin](/fr/guide/creation-functions/combination/forkJoin) | Traitement étape par étape, intégration d'événements multiples, synchronisation des entrées de formulaires, traitement parallèle, etc.APIAttente de l'achèvement d'un appel |
| **[Systèmes de sélection et de division](/fr/guide/creation-functions/selection/)** | Un système dans lequel plusieursObservableA partir de.1Sélectionner un ou1unObservableSéparer en plusieurs systèmes | [race](/fr/guide/creation-functions/selection/race), [partition](/fr/guide/creation-functions/selection/partition) | Concurrence de sources de données multiples, succès/Processus de ramification en cas d'échec |
| **[Système de branchement conditionnel](/fr/guide/creation-functions/conditional/)** | basé sur des conditionsObservablesélectionnées ou générées dynamiquement lors de l'inscription | [iif](/fr/guide/creation-functions/conditional/iif), [defer](/fr/guide/creation-functions/conditional/defer) | Processus de ramification basé sur l'état de connexion, la création dynamique, l'évaluation différéeObservablecréation dynamique, évaluation différée |
| **[Système de contrôle](/fr/guide/creation-functions/control/)** | ObservableContrôle de la synchronisation de l'exécution et de la gestion des ressources | [scheduled](/fr/guide/creation-functions/control/scheduled), [using](/fr/guide/creation-functions/control/using) | Contrôle de la synchronisation de l'exécution par le planificateur, gestion du cycle de vie des ressources, prévention des fuites de mémoire |

> [!TIP]
> **Séquence d'apprentissage**
>
> Il est recommandé aux nouveaux apprenants d'étudier dans l'ordre suivant
> 1. **Système de création de base** - RxJSLes fonctions de base de
> 2. **Système couplé** - Les bases de la gestion de flux multiples
> 3. **HTTPSystèmes de communication** - PratiqueAPITravailler avec des flux multiples en conjonction les uns avec les autres
> 4. Autres catégories - Apprentissage selon les besoins

## Pipeable Operator Correspondance avec

Nombreux sont ceux quiCreation Functionsont un correspondantPipeable Operatorcorrespondant. Lorsqu'il est utilisé dans un pipeline,`~With`les opérateurs de système sont utilisés.

| Creation Function | Pipeable Operator | Remarque. |
|-------------------|-------------------|------|
| `concat(a$, b$)` | `a$.pipe(`**[concatWith](/fr/guide/operators/combination/concatWith)**`(b$))` | RxJS 7+ |
| `merge(a$, b$)` | `a$.pipe(`**[mergeWith](/fr/guide/operators/combination/mergeWith)**`(b$))` | RxJS 7+ |
| `zip(a$, b$)` | `a$.pipe(`**[zipWith](/fr/guide/operators/combination/zipWith)**`(b$))` | RxJS 7+ |
| `combineLatest([a$, b$])` | `a$.pipe(`**[combineLatestWith](/fr/guide/operators/combination/combineLatestWith)**`(b$))` | RxJS 7+ |
| `race(a$, b$)` | `a$.pipe(`**[raceWith](/fr/guide/operators/combination/raceWith)**`(b$))` | RxJS 7+ |

> [!NOTE]
> RxJS 7Après,**[concatWith](/fr/guide/operators/combination/concatWith)**, **[mergeWith](/fr/guide/operators/combination/mergeWith)**, **[zipWith](/fr/guide/operators/combination/zipWith)**, **[combineLatestWith](/fr/guide/operators/combination/combineLatestWith)**, **[raceWith](/fr/guide/operators/combination/raceWith)** tels que`~With`ont été ajoutés,Pipeable OperatorIl est maintenant plus facile de les utiliser comme

## Ce qui devrait être utilisé？

Creation FunctionetPipeable Operatordépend du contexte.

### Creation Function Il est recommandé d'utiliser

Un système dans lequel plusieursObservablelorsqu'on opère au même niveau,Creation Functionafin de préserver la concision du code.

```

typescript.
// ✅ Fusionner plusieurs Observable au même niveau
const combined$ = merge(
  
  fromEvent(button2, 'click'), fromEvent(button3, 'click'), fromEvent(button4, 'click')
  fromEvent(button3, 'click')
) ;

```

### Pipeable Operator Il est recommandé d'utiliser

Si vous ajoutez des opérations dans le cadre d'un pipeline,Pipeable Operatorpour clarifier le flux du processus.

```

typescript.
// ✅ Joindre en tant que partie d'un pipeline
const result$ = source$.pipe(
  map(x => x * 2),.
  mergeWith(other$),.
  filter(x => x > 10)
) ;
```

## Résumé.

- **Creation Functions** : fonctions pour créer et combiner des Observable.
- **Pipeable Operators** : fonctions pour convertir des Observable existants.
- Les Creation Function se répartissent en sept catégories
  1.**Systèmes de création de base** : création d'Observable basés sur les données, les tableaux, les événements et le temps.
  2.**Générateurs de boucles** : représentent des processus itératifs sous forme d'Observable.
  3.**Système de communication HTTP** : traiter la communication HTTP en tant qu'Observable.
  4.**Système de combinaison** : combinez plusieurs éléments en un seul.
  5.**Systèmes de sélection et de division** : sélectionner ou diviser.
  6.**Systèmes de branchement conditionnel** : générer dynamiquement en fonction des conditions.
  7. **Systèmes de contrôle** : contrôle du temps d'exécution et de la gestion des ressources.
- Pipeable Operator dans la famille `~With` des pipelines.
- Chaque catégorie contient plusieurs fonctions qui peuvent être utilisées en fonction de l'application.

## Prochaines étapes.

Pour en savoir plus sur chaque catégorie, suivez les liens ci-dessous.

1.**[Fonctions de création de base Fonctions de création](/fr/guide/creation-functions/basic/)** - of, from, fromEvent, interval, timer
2.**[Génération de boucles Fonctions de Création](/fr/guide/creation-functions/loop/)** - range, generate
3. **[Fonctions de création de communication HTTP](/fr/guide/creation-functions/http-communication/)** - ajax, fromFetch
4.**[Fonctions Creation Function](/fr/guide/creation-functions/combination/)** - concat, merge, combineLatest, zip, forkJoin
5.**[Fonctions de création de sélection et de partition](/fr/guide/creation-functions/selection/)** - race, partition
6.**[Fonctions de Création de Branchements Conditionnels](/fr/guide/creation-functions/conditional/)** - iif, defer
7.**[Fonctions de contrôle Fonctions de création](/fr/guide/creation-functions/control/)** - scheduled, using

Sur chaque page, vous pouvez en apprendre davantage sur le fonctionnement détaillé des Creation Function et sur des exemples pratiques.

## Ressources de référence.

- [Documentation officielle RxJS - Creation Function](https://rxjs.dev/guide/operators#creation-operators-list)
- [Apprendre RxJS - Opérateurs de création](https://www.learnrxjs.io/learn-rxjs/operators/creation)

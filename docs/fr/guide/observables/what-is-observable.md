---
description: "Observable est un concept central de RxJS et représente un flux de données qui se produit dans le temps. La différence avec Promise, le mécanisme d'abonnement et de désabonnement, la distinction entre cold et hot et la gestion du cycle de vie sont expliqués à l'aide d'exemples de code pratiques."
---

# Qu'est-ce qu'un Observable ?

[📘 Documentation officielle RxJS : Observable](https://rxjs.dev/api/index/class/Observable)

Observable dans RxJS est un composant de base qui représente "un flux de données qui se produit dans le temps" et est conçu sur la base du modèle Observer pour gérer le traitement asynchrone et événementiel d'une manière unifiée.

## Rôle de l'Observable

Un Observable agit comme un "producteur de données" qui publie de multiples valeurs au fil du temps. En revanche, un Observer agit comme un "consommateur" et s'abonne aux valeurs via `subscribe()`.

Dans l'exemple suivant, nous créons un **Observable (producteur)** appelé `observable$` et un **Observer (consommateur)** s'abonne et reçoit des valeurs.

```ts
import { Observable } from 'rxjs';

// Création d'un Observable (producteur)
const observable$ = new Observable<number>(subscriber => {
  // Logique à exécuter lors de l'abonnement
  subscriber.next(1);
  subscriber.next(2);
  subscriber.complete();
});

// L'Observer (consommateur) s'abonne
observable$.subscribe({
  next: value => console.log('Valeur suivante:', value),
  error: err => console.error('Erreur:', err),
  complete: () => console.log('Terminé')
});

// Sortie :
// Valeur suivante: 1
// Valeur suivante: 2
// Terminé
```

> [!NOTE]
> La fonction passée en argument à `new Observable(function)` définit la **logique à exécuter lorsque l'Observable est souscrit**. La fonction elle-même n'est pas le producteur ; c'est l'Observable dans son ensemble qui est le producteur.

## Types de notifications

Observable envoie les trois types de notifications suivants à Observer :

- `next` : notification d'une valeur
- `error` : notification lorsqu'une erreur se produit (aucune autre notification n'est envoyée)
- `complete` : notification d'achèvement réussi

Pour plus d'informations, voir la section [Observer dans "Cycle de vie de l'Observable"](./observable-lifecycle.md#_2-observer-observer).

## Différence entre Observable et Promise

| Caractéristique | Observable | Promise |
|---|---|---|
| Valeurs multiples | ◯ | ×(Unique seulement) |
| Annulable | ◯(`unsubscribe()`) | × |
| Exécution paresseuse | ◯ | ◯ |
| Sync/Async | Les deux | Async seulement |

La plus grande différence entre Observable et Promise est de savoir "s'il peut gérer des valeurs multiples" et "s'il peut annuler au milieu".
Promise est adapté au traitement asynchrone ponctuel, tandis que Observable est plus adapté aux "données asynchrones continues" telles que les flux d'événements.

Observable est également important en termes de gestion des ressources, comme la prévention des fuites de mémoire et l'arrêt des communications inutiles, puisque les abonnements peuvent être annulés au milieu d'un processus par `unsubscribe()`.

D'un autre côté, Promise est largement adopté dans l'API standard et peut être écrit de manière intuitive en combinaison avec `async/await`. Il est souhaitable d'utiliser les deux en fonction de l'application.

## Distinction entre Cold et Hot

Il existe deux types d'Observable dans RxJS : "Cold" et "Hot".

- **Cold Observable** : Chaque abonné a son propre flux de données, qui commence à s'exécuter lorsqu'il est abonné. (par exemple, `of()`, `from()`, `fromEvent()`, `ajax()`)
- **Hot Observable** : Les abonnés partagent le même flux de données et les données continuent de circuler, qu'ils soient abonnés ou non. (par exemple, `Subject`, Observable multicast avec `share()`)

Cette distinction a un impact significatif sur le partage des données et l'efficacité des ressources.
Pour plus d'informations, voir la section ["Cold Observable et Hot Observable"](./cold-and-hot-observables.md).

## Observable et Pipeline

La vraie valeur d'un Observable est réalisée en le combinant avec des opérateurs en utilisant la méthode `pipe()`.

```ts
import { of } from 'rxjs';
import { map, filter } from 'rxjs';

const numbers$ = of(1, 2, 3, 4, 5);
numbers$.pipe(
  filter(n => n % 2 === 0), // Ne passer que les nombres pairs
  map(n => n * 10)          // Multiplier par 10
).subscribe(value => console.log(value));
// Sortie : 20, 40
```

## Cycle de vie de l'Observable

Observable a le cycle de vie suivant :

1. **Création** - création d'une instance d'Observable
2. **Subscribe** - commence à recevoir des données par `subscribe()`
3. **Exécution** - publier des données (`next`), une erreur (`error`), ou l'achèvement (`complete`)
4. **Unsubscribe** - termine l'abonnement avec `unsubscribe()`

Il est important de se désabonner des abonnements Observable qui ne sont plus nécessaires afin d'éviter les fuites de ressources.
Pour plus de détails, voir la section ["Cycle de vie de l'Observable"](./observable-lifecycle.md).

## Où utiliser Observable

- Événements de l'interface utilisateur (clics, défilement, opérations au clavier, etc.)
- Requêtes HTTP
- Traitement temporel (intervalles et minuteries)
- WebSocket et communication en temps réel
- Gestion de l'état de l'application

## Résumé

Observable est la base d'une gestion flexible et unifiée des données asynchrones. En tant que concept central de ReactiveX (RxJS), il fournit une représentation concise des traitements asynchrones complexes et des flux d'événements.

---
description: "combineLatestAll est un opérateur qui prend un Observable d'ordre supérieur (Observable d'Observable) et combine la dernière valeur de chacun d'entre eux une fois que tous les Observable internes se sont déclenchés au moins une fois."
---

# combineLatestAll - combine les dernières valeurs de l'Observable interne

L'opérateur `combineLatestAll` prend un **Observable d'ordre supérieur** (Observable d'Observables),
**Une fois que tous les Observable internes se sont déclenchés au moins une fois**, les **dernières valeurs** de chacun sont combinées et affichées sous la forme d'un **tableau**.

## 🔰 Syntaxe de base et utilisation

```ts
import { interval, of } from 'rxjs';
import { combineLatestAll, take } from 'rxjs';

// 3Deux internesObservableavecHigher-order Observable
const higherOrder$ = of(
  interval(1000).pipe(take(3)), // 0, 1, 2
  interval(500).pipe(take(4)),  // 0, 1, 2, 3
  interval(2000).pipe(take(2))  // 0, 1
);

// Tous les internesObservableont au moins1feux une fois, combiner les dernières valeurs
higherOrder$
  .pipe(combineLatestAll())
  .subscribe(values => console.log(values));

// Sortie:
// [1, 3, 0] ← Lorsque tous les événements ont été déclenchés au moins une fois1Lorsque tous les coups ont été tirés au moins une fois (après2secondes plus tard)
// [2, 3, 0] ← 1Lorsque le deuxièmeObservableest tiré (après 2 secondes)2tire (après3secondes plus tard)
// [2, 3, 1] ← 3Lorsque le deuxièmeObservableest tiré (après 2 secondes)1tire (après4secondes plus tard)
```

- Collecter les Observables internes lorsque l'Observable d'ordre supérieur est **complet**.
- Commencez à combiner lorsque tous les Observable internes se sont déclenchés** au moins une fois.
- Chaque fois qu'un Observable interne émet une valeur, **combinez** toutes les dernières valeurs et **sorties**.

[🌐 Official RxJS documentation - `combineLatestAll`](https://rxjs.dev/api/index/function/combineLatestAll)

## 💡 Modèle d'utilisation typique.

- **Combiner les derniers résultats de plusieurs appels API**
- **Synchroniser les dernières valeurs de plusieurs entrées de formulaire**
- **Intégrer plusieurs sources de données en temps réel**

## 🧠 Exemples de code pratiques

Exemple de combinaison des derniers résultats de plusieurs appels API

```ts
import { of, timer, Observable } from 'rxjs';
import { map, combineLatestAll, take } from 'rxjs';

const output = document.createElement('div');
document.body.appendChild(output);

// 3Deux simulationsAPICréer un appel (Higher-order Observable)
const apiCalls$: Observable<Observable<string>> = of(
  // API 1: Informations sur l'utilisateur (1Effectué en quelques secondes,3mise à jour une fois)
  timer(0, 1000).pipe(
    take(3),
    map(n => `Utilisateur: User${n}`)
  ),
  // API 2: Nombre de notifications (0.5Effectué en quelques secondes,4mise à jour une fois)
  timer(0, 500).pipe(
    take(4),
    map(n => `Notifications: ${n}Nombre de notifications`)
  ),
  // API 3: Statut2Effectué en quelques secondes,2mise à jour une fois)
  timer(0, 2000).pipe(
    take(2),
    map(n => n === 0 ? 'Statut: Hors ligne' : 'Statut: En ligne')
  )
);

// TousAPIDernière valeur combinée des appels
apiCalls$
  .pipe(combineLatestAll())
  .subscribe(values => {
    output.innerHTML = '<strong>Dernier statut:</strong><br>';
    values.forEach((value, index) => {
      const item = document.createElement('div');
      item.textContent = `${index + 1}. ${value}`;
      output.appendChild(item);
    });
  });
```

- Trois appels d'API sont exécutés en parallèle.
- Une fois qu'ils ont tous été exécutés au moins une fois**, les résultats combinés sont affichés.
- Chaque fois qu'une API est mise à jour, la **dernière combinaison** est affichée.

## 🔄 Creation Function associée

`combineLatestAll` est principalement utilisée pour aplatir les Observable d'ordre supérieur,
Utilisez la **Creation Function** `combineLatest` pour les combinaisons Observables normales.

```ts
import { combineLatest, interval } from 'rxjs';

// Creation FunctionEdition (utilisation plus générale)
const combined$ = combineLatest([
  interval(1000),
  interval(500),
  interval(2000)
]);

combined$.subscribe(console.log);
```

Voir [Chapitre 3 Creation Function - combineLatest](/fr/guide/creation-functions/combination/combineLatest).

## 🔄 Opérateurs apparentés.

| Opérateur. | Description. |
|---|---|
| [mergeAll](./mergeAll) | Souscrire à tous les Observable internes en parallèle. |
| [concatAll](./concatAll) | S'abonner à des Observable internes en séquence. |
| [switchAll](./switchAll) | Passer à un nouvel Observable interne. |
| [zipAll](./zipAll) | Apparier les valeurs de chaque Observable interne dans l'ordre correspondant. |

## ⚠️ Notes.

### L'Observable d'ordre supérieur doit être complété.

`combineLatestAll` attend la collection d'Observables internes **jusqu'à** ce que l'Observable d'ordre supérieur (Observable externe) soit **complété**.

#### ❌ L'Observable d'ordre supérieur n'est pas terminé, donc rien n'est produit.

```ts
interval(1000).pipe(
  map(() => of(1, 2, 3)),
  combineLatestAll()
).subscribe(console.log); // Rien n'est édité
```

#### ✅ Prendre pour compléter

```ts
interval(1000).pipe(
  take(3), // 3Terminé en une seule session
  map(() => of(1, 2, 3)),
  combineLatestAll()
).subscribe(console.log);
```

### Tous les Observable internes doivent se déclencher au moins une fois

Aucune valeur n'est émise tant que tous les Observable internes n'ont pas été déclenchés **au moins une fois**.

```ts
// 1Si l'un des appels internesObservableRien n'est édité s'il y a
of(
  of(1, 2, 3),
  NEVER // Ne se déclenche jamais pour toujours.
).pipe(
  combineLatestAll()
).subscribe(console.log); // Rien n'est édité
```

### Utilisation de la mémoire.

Notez l'utilisation de la mémoire s'il y a beaucoup d'Observable internes, car les **dernières valeurs de tous les Observable internes sont conservées en mémoire**.

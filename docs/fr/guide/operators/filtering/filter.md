---
description: "L'opérateur filter trie les valeurs d'un flux sur la base d'une fonction conditionnelle spécifiée, ne laissant passer que les valeurs qui satisfont la condition. Il peut être utilisé comme une fonction de garde de type (prédicat de type) dans TypeScript, et explique également la différence entre lui et le buffer, ainsi que les mises en garde relatives à la transformation d'une fonction de prédicat en fonction pure. Cette section explique également la différence entre les tampons et les fonctions pures."
---

# filter - ne laisser passer que les valeurs qui correspondent aux conditions

L'opérateur `filter` trie les valeurs d'un flux en fonction d'une fonction conditionnelle spécifiée et ne laisse passer que les valeurs qui remplissent la condition.

## 🔰 Syntaxe de base et utilisation

```ts
import { from } from 'rxjs';
import { filter } from 'rxjs';

const numbers$ = from([1, 2, 3, 4, 5, 6, 7, 8, 9, 10]);

numbers$.pipe(
  filter(n => n % 2 === 0)
).subscribe(console.log);
// Résultats: 2, 4, 6, 8, 10
```

- Seules les valeurs correspondant à la condition sont passées.
- Fonctionne comme `Array.prototype.filter()` sur les tableaux, mais est séquentiel sur l'Observable.

[🌐 Documentation officielle de RxJS - `filter`](https://rxjs.dev/api/operators/filter)

## 💡 Modèle d'utilisation typique.

- Validation des valeurs d'entrée d'un formulaire
- Autoriser uniquement les données d'un type ou d'une structure spécifique
- Filtrage des événements des capteurs et des données de flux

## 🧠 Exemples de codes pratiques (avec interface utilisateur)

Liste en temps réel uniquement si le nombre saisi est pair.

```ts
import { fromEvent } from 'rxjs';
import { map, filter } from 'rxjs';

const title = document.createElement('h3');
title.innerHTML = 'filter Exemples pratiques:';
document.body.appendChild(title);

// Création de champs de saisie
const input = document.createElement('input');
input.type = 'number';
input.placeholder = 'Saisir des valeurs numériques';
input.style.marginBottom = '10px';
document.body.appendChild(input);

// Création d'une zone de sortie
const output = document.createElement('div');
document.body.appendChild(output);

// Flux d'événements d'entrée
fromEvent(input, 'input')
  .pipe(
    map((e) => parseInt((e.target as HTMLInputElement).value, 10)),
    filter((n) => !isNaN(n) && n % 2 === 0)
  )
  .subscribe((evenNumber) => {
    const item = document.createElement('div');
    item.textContent = `Détection des nombres pairs: ${evenNumber}`;
    output.prepend(item);
  });

```

- Affiché uniquement dans la sortie si le nombre est pair.
- Les entrées impaires ou invalides sont ignorées.

> [!WARNING] 本番コードでの注意

> L'exemple ci-dessus omet la désinscription de `fromEvent` pour simplifier l'explication. Dans le code réel, utilisez `takeUntil(destroy$)`, `take(N)` ou `Subscription.unsubscribe()` pour gérer explicitement le cycle de vie. Plus d'informations : [Surmonter les difficultés : gestion du cycle de vie](/fr/guide/overcoming-difficulties/lifecycle-management.md)

## 🔍 Différences avec le buffer

| Opérateur | Fonctionnement | Sortie. |
|---|---|---|
| filter`. | Rejeter les valeurs qui ne **correspondent** pas à la condition. | Valeurs individuelles `T`. |
| `buffer`. | Stocke** les valeurs dans un tableau**. | Tableau `T[]` |

```ts
import { interval } from 'rxjs';
import { filter, buffer, take } from 'rxjs';

const source$ = interval(1000).pipe(take(5)); // 0, 1, 2, 3, 4

// filter - Seules les valeurs correspondant aux conditions sont transmises
source$.pipe(
  filter(x => x % 2 === 0)
).subscribe(x => {
  console.log('filter:', x);
  // Résultats: filter: 0
  // Résultats: filter: 2
  // Résultats: filter: 4
});

// buffer - Stockage des valeurs sous forme de tableau
source$.pipe(
  buffer(interval(2500))
).subscribe(arr => {
  console.log('buffer:', arr);
  // Résultats: buffer: [0, 1]
  // Résultats: buffer: [2, 3, 4]
});
```

## ⚠️ Notes.

### 1. les fonctions prédicats doivent être des fonctions pures

Les fonctions prédicats ayant des effets secondaires peuvent entraîner un comportement inattendu lorsque le flux est réabonné.

```ts
// ❌ Mauvais exemple: Effets secondaires Oui
let counter = 0;
source$.pipe(
  filter(x => {
    counter++; // Effet secondaire
    return x > 10;
  })
).subscribe();

// ✅ Bon exemple: Fonction pure
source$.pipe(
  filter(x => x > 10)
).subscribe();
```

### 2. utiliser comme fonction de protection de type

Vous pouvez l'écrire pour retourner un prédicat de type TypeScript (`x is T`) pour réduire le type après avoir passé `filter`.

```ts
import { Observable, of, filter } from 'rxjs';

interface User {
  id: number;
  name: string;
  email?: string;
}

const users$: Observable<User> = of(
  { id: 1, name: 'Alice', email: 'alice@example.com' },
  { id: 2, name: 'Bob' }
);

// Utilisée comme fonction de protection de type
users$.pipe(
  filter((user): user is User & { email: string } => user.email !== undefined)
).subscribe(user => {
  console.log(user.email.toLowerCase()); // email n'est pas une fonction de protection de type string Est déduite comme un type
});
```

> [!TIP] 型ガードの効果

> En retournant le prédicat de type `user is User & { email : string }`, `user` après `filter` fait de `email` une propriété obligatoire. Des appels comme `user.email.toLowerCase()` peuvent être écrits sans erreur de type.

## 📚 Opérateurs apparentés.

- [take](/fr/guide/operators/filtering/take) - seules les N premières valeurs sont prises.
- [first](/fr/guide/operators/filtering/first) - ne prend que la première valeur (peut aussi être conditionnel).
- [distinct](/fr/guide/operators/filtering/distinct) - exclut les valeurs en double.
- [distinctUntilChanged](/fr/guide/operators/filtering/distinctUntilChanged) - Exclut la même valeur que la dernière.

## Résumé.

L'opérateur `filter` est l'outil de filtrage le plus basique de RxJS.

- ✅ Seules les valeurs qui correspondent aux conditions sont passées.
- ✅ Peut être utilisé de la même manière que `.filter()` pour les tableaux.
- ✅ Peut également être utilisé comme une protection de type TypeScript.
- ⚠️ Les fonctions de prédicat doivent être des fonctions pures.
- ⚠️ Nom similaire mais usage différent de `buffer` (valeurs individuelles vs. tableaux).

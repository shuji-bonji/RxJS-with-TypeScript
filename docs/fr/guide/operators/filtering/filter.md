---
description: "L'opérateur filter sélectionne les valeurs d'un flux sur la base d'une fonction de condition spécifiée, ne transmettant que les valeurs qui remplissent la condition. Il s'agit d'un opérateur de filtrage qui exclut les données inutiles pour rationaliser le flux, comme la validation des entrées de formulaire, l'extraction de données avec des conditions spécifiques et l'exclusion des valeurs null ou undefined. Il peut également être utilisé comme un type guard TypeScript."
---

# filter - Filtrer par Condition

L'opérateur `filter` sélectionne les valeurs dans un flux basé sur une fonction de condition spécifiée et ne laisse passer que les valeurs qui satisfont la condition.

## 🔰 Syntaxe de base et utilisation

```ts
import { from } from 'rxjs';
import { filter } from 'rxjs';

const numbers$ = from([1, 2, 3, 4, 5, 6, 7, 8, 9, 10]);

numbers$.pipe(
  filter(n => n % 2 === 0)
).subscribe(console.log);
// Sortie: 2, 4, 6, 8, 10
```

- Seules les valeurs correspondant à la condition passent.
- Similaire à `Array.prototype.filter()`, mais traité séquentiellement sur un Observable.

[🌐 Documentation officielle RxJS - `filter`](https://rxjs.dev/api/operators/filter)

> [!WARNING] Attention en code de production
> L'exemple ci-dessus omet la désinscription de `fromEvent` pour simplifier l'explication. Dans du code réel, gérez explicitement le cycle de vie avec `takeUntil(destroy$)`, `take(N)`, ou `Subscription.unsubscribe()`. Détails : [Surmonter les difficultés : gestion du cycle de vie](/fr/guide/overcoming-difficulties/lifecycle-management.md)

## 💡 Patterns d'utilisation typiques

- Validation des valeurs d'entrée de formulaire
- Autoriser uniquement les données d'un type ou d'une structure spécifique
- Filtrage des événements de capteurs ou des données de flux

## 🧠 Exemple de code pratique (avec UI)

Affiche en temps réel dans une liste uniquement lorsque le nombre saisi est pair.

```ts
import { fromEvent } from 'rxjs';
import { map, filter } from 'rxjs';

const title = document.createElement('h3');
title.innerHTML = 'Exemple pratique de filter :';
document.body.appendChild(title);

// Création du champ de saisie
const input = document.createElement('input');
input.type = 'number';
input.placeholder = 'Entrez un nombre';
input.style.marginBottom = '10px';
document.body.appendChild(input);

// Création de la zone de sortie
const output = document.createElement('div');
document.body.appendChild(output);

// Flux d'événements de saisie
fromEvent(input, 'input')
  .pipe(
    map((e) => parseInt((e.target as HTMLInputElement).value, 10)),
    filter((n) => !isNaN(n) && n % 2 === 0)
  )
  .subscribe((evenNumber) => {
    const item = document.createElement('div');
    item.textContent = `Nombre pair détecté : ${evenNumber}`;
    output.prepend(item);
  });

```

- Seuls les nombres pairs sont affichés dans la sortie.
- Les nombres impairs ou les entrées invalides sont ignorés.

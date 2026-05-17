---
description: "L'opérateur isEmpty détermine si un Observable s'est terminé sans émettre de valeur. Il est utilisé pour détecter les données vides, les branchements conditionnels et les vérifications d'existence de données. Détection quand un résultat est vide dans filter() et implémentation type-safe en TypeScript avec des exemples de code pratiques."
---

# isEmpty - Déterminer si le flux est vide

L'opérateur `isEmpty` émet `true` si l'Observable se termine **sans émettre une seule valeur**.
S'il émet ne serait-ce qu'une valeur, il émet `false` et termine.

## 🔰 Syntaxe et fonctionnement de base

```ts
import { of, EMPTY } from 'rxjs';
import { isEmpty } from 'rxjs';

EMPTY.pipe(isEmpty()).subscribe(console.log); // Sortie : true
of(1).pipe(isEmpty()).subscribe(console.log); // Sortie : false
```

[🌐 Documentation officielle RxJS - isEmpty](https://rxjs.dev/api/index/function/isEmpty)

## 💡 Cas d'utilisation typiques

- Lorsque vous voulez déterminer si un résultat de filtrage ou de recherche est vide
- Lorsque vous voulez générer une erreur ou basculer vers un autre traitement si le résultat est vide

```ts
import { from } from 'rxjs';
import { filter, isEmpty } from 'rxjs';

from([1, 3, 5])
  .pipe(
    filter((x) => x % 2 === 0),
    isEmpty()
  )
  .subscribe((result) => {
    console.log('Vide ou non :', result);
  });

// Sortie :
// Vide ou non : true
```

## 🧪 Exemples de code pratiques (avec interface utilisateur)

### ✅ 1. Déterminer si le résultat est vide

```ts
import { from } from 'rxjs';
import { filter, isEmpty } from 'rxjs';

const container = document.createElement('div');
container.innerHTML = '<h3>Exemple de l\'opérateur isEmpty :</h3>';
document.body.appendChild(container);

const checkButton = document.createElement('button');
checkButton.textContent = 'Vérifier si contient des nombres pairs';
container.appendChild(checkButton);

const output = document.createElement('div');
output.style.marginTop = '10px';
output.style.padding = '10px';
output.style.border = '1px solid #ccc';
container.appendChild(output);

checkButton.addEventListener('click', () => {
  from([1, 3, 5])
    .pipe(
      filter((x) => x % 2 === 0),
      isEmpty()
    )
    .subscribe((isEmptyResult) => {
      output.textContent = isEmptyResult
        ? 'Aucun nombre pair n\'a été inclus.'
        : 'Contient un nombre pair.';
      output.style.color = isEmptyResult ? 'red' : 'green';
    });
});
```

### ✅ 2. Vérifier si les résultats de recherche de l'utilisateur sont vides

```ts
import { fromEvent, of, from } from 'rxjs';
import { debounceTime, switchMap, map, filter, isEmpty, delay } from 'rxjs';

const searchContainer = document.createElement('div');
searchContainer.innerHTML = '<h3>Vérification des résultats de recherche avec isEmpty :</h3>';
document.body.appendChild(searchContainer);

const input = document.createElement('input');
input.placeholder = 'Entrez un terme de recherche';
input.style.marginBottom = '10px';
searchContainer.appendChild(input);

const resultBox = document.createElement('div');
resultBox.style.padding = '10px';
resultBox.style.border = '1px solid #ccc';
searchContainer.appendChild(resultBox);

const mockData = ['apple', 'banana', 'orange', 'grape'];

fromEvent(input, 'input')
  .pipe(
    debounceTime(300),
    map((e) => (e.target as HTMLInputElement).value.trim().toLowerCase()),
    filter((text) => text.length > 0),
    switchMap((query) =>
      of(mockData).pipe(
        delay(300),
        map((list) => list.filter((item) => item.includes(query))),
        switchMap((filtered) => from(filtered).pipe(isEmpty()))
      )
    )
  )
  .subscribe((noResults) => {
    resultBox.textContent = noResults
      ? 'Aucun résultat trouvé'
      : 'Correspondance trouvée';
    resultBox.style.color = noResults ? 'red' : 'green';
  });
```

> [!WARNING] Attention en code de production
> L'exemple ci-dessus omet la désinscription de `fromEvent` pour simplifier l'explication. Dans du code réel, gérez explicitement le cycle de vie avec `takeUntil(destroy$)`, `take(N)`, ou `Subscription.unsubscribe()`. Détails : [Surmonter les difficultés : gestion du cycle de vie](/fr/guide/overcoming-difficulties/lifecycle-management.md)

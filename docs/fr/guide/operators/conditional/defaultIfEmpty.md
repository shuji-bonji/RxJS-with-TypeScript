---
description: "L'opérateur defaultIfEmpty renvoie une valeur par défaut si l'Observable n'émet pas de valeur. Des cas d'utilisation pratiques et une mise en œuvre type-safe en TypeScript sont décrits, y compris la gestion des réponses API vides, la complétion de la valeur initiale et le repli pour les résultats de recherche manquants."
---

# defaultIfEmpty - Valeur par défaut si le flux est vide

L'opérateur `defaultIfEmpty` est un **opérateur qui émet une valeur par défaut spécifiée si l'Observable se termine sans émettre de valeur**.
Il est utilisé pour traiter les tableaux vides et les résultats d'API vides.

## 🔰 Syntaxe et comportement de base

```ts
import { from } from 'rxjs';
import { defaultIfEmpty } from 'rxjs';

from([]).pipe(
  defaultIfEmpty('Pas de valeur')
).subscribe(console.log);

// Sortie :
// Pas de valeur
```

Dans cet exemple, `defaultIfEmpty` affichera `'Pas de valeur'` pour un tableau vide rendu Observable avec `from`.

[🌐 Documentation officielle RxJS - defaultIfEmpty](https://rxjs.dev/api/index/function/defaultIfEmpty)

## 💡 Cas d'utilisation typiques

- Si l'utilisateur n'a rien saisi
- Si l'API renvoie un résultat vide
- Si aucune des valeurs ne satisfait aux conditions

Ceci est utilisé pour **compléter une situation où « rien n'a été retourné »**.

```ts
import { of, EMPTY } from 'rxjs';
import { defaultIfEmpty, delay } from 'rxjs';

function mockApiCall(hasData: boolean) {
  return hasData
    ? of(['A', 'B', 'C']).pipe(delay(500))
    : EMPTY.pipe(delay(500));
}

mockApiCall(false)
  .pipe(defaultIfEmpty('Pas de données'))
  .subscribe(console.log);

// Sortie :
// Pas de données
```

## 🧪 Exemple de code pratique (avec interface utilisateur)

### ✅ 1. Utilisé pour déterminer si un tableau est vide

```ts
import { from } from 'rxjs';
import { defaultIfEmpty } from 'rxjs';

// Construction de l'interface utilisateur
const container = document.createElement('div');
container.innerHTML = '<h3>Exemple de l\'opérateur defaultIfEmpty :</h3>';
document.body.appendChild(container);

const emptyBtn = document.createElement('button');
emptyBtn.textContent = 'Traiter un tableau vide';
container.appendChild(emptyBtn);

const nonEmptyBtn = document.createElement('button');
nonEmptyBtn.textContent = 'Traiter un tableau non vide';
container.appendChild(nonEmptyBtn);

const result = document.createElement('div');
result.style.marginTop = '10px';
result.style.padding = '10px';
result.style.border = '1px solid #ccc';
container.appendChild(result);

emptyBtn.addEventListener('click', () => {
  result.textContent = 'Traitement en cours...';
  from([]).pipe(
    defaultIfEmpty('Pas de données')
  ).subscribe(value => {
    result.textContent = `Résultat : ${value}`;
  });
});

nonEmptyBtn.addEventListener('click', () => {
  result.textContent = 'Traitement en cours...';
  from([1, 2, 3]).pipe(
    defaultIfEmpty('Pas de données')
  ).subscribe(value => {
    result.textContent = `Résultat : ${value}`;
  });
});
```

### ✅ 2. Compléter les valeurs par défaut pour les résultats API vides

```ts
import { of, EMPTY } from 'rxjs';
import { defaultIfEmpty, delay } from 'rxjs';

function mockApiCall(hasData: boolean) {
  return hasData
    ? of([
        { id: 1, name: 'Item 1' },
        { id: 2, name: 'Item 2' },
      ]).pipe(delay(1000))
    : EMPTY.pipe(delay(1000));
}

const apiContainer = document.createElement('div');
apiContainer.innerHTML = '<h3>Traitement des résultats API avec defaultIfEmpty :</h3>';
document.body.appendChild(apiContainer);

const dataBtn = document.createElement('button');
dataBtn.textContent = 'Avec données';
dataBtn.style.marginRight = '10px';
apiContainer.appendChild(dataBtn);

const emptyBtn2 = document.createElement('button');
emptyBtn2.textContent = 'Sans données';
apiContainer.appendChild(emptyBtn2);

const output = document.createElement('div');
output.style.marginTop = '10px';
output.style.padding = '10px';
output.style.border = '1px solid #ccc';
apiContainer.appendChild(output);

dataBtn.addEventListener('click', () => {
  output.textContent = 'Récupération en cours...';
  mockApiCall(true)
    .pipe(defaultIfEmpty('Aucune donnée trouvée'))
    .subscribe({
      next: (val) => {
        if (Array.isArray(val)) {
          const ul = document.createElement('ul');
          val.forEach((item) => {
            const li = document.createElement('li');
            li.textContent = `${item.id}: ${item.name}`;
            ul.appendChild(li);
          });
          output.innerHTML = '<h4>Résultat de la récupération :</h4>';
          output.appendChild(ul);
        } else {
          output.textContent = val;
        }
      },
    });
});

emptyBtn2.addEventListener('click', () => {
  output.textContent = 'Récupération en cours...';
  mockApiCall(false)
    .pipe(defaultIfEmpty('Aucune donnée trouvée'))
    .subscribe({
      next: (val) => {
        output.textContent = val.toString();
      },
    });
});

```

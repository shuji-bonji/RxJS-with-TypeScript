---
description: "L'opérateur elementAt est un opérateur de filtrage RxJS qui récupère uniquement la valeur à une position d'index spécifiée. Il fonctionne de manière similaire à l'accès par index des tableaux."
---

# elementAt - Récupérer la valeur à un index spécifié

L'opérateur `elementAt` récupère **uniquement la valeur à la position d'index spécifiée** d'un Observable et termine immédiatement le flux. Il fonctionne de manière similaire à `array[index]` pour les tableaux.

## 🔰 Syntaxe de base et utilisation

```ts
import { from } from 'rxjs';
import { elementAt } from 'rxjs';

const numbers$ = from([10, 20, 30, 40, 50]);

numbers$.pipe(
  elementAt(2)
).subscribe(console.log);
// Sortie: 30 (valeur à l'index 2)
```

**Flux d'opération** :
1. 10 (index 0) → Ignoré
2. 20 (index 1) → Ignoré
3. 30 (index 2) → Émis puis terminé
4. 40, 50 ne sont pas évalués

[🌐 Documentation officielle RxJS - `elementAt`](https://rxjs.dev/api/operators/elementAt)

## 💡 Patterns d'utilisation typiques

- **Pagination** : Récupérer le premier élément d'une page spécifique
- **Récupération de données ordonnées** : Récupérer le N-ième événement ou message
- **Test et débogage** : Vérifier la valeur à une position spécifique
- **Accès type tableau** : Traiter un Observable comme un tableau

## 🧠 Exemple de code pratique 1 : Compte à rebours d'événements

Un exemple qui exécute une action au N-ième clic.

```ts
import { fromEvent } from 'rxjs';
import { elementAt, map } from 'rxjs';

// Création de l'UI
const output = document.createElement('div');
output.innerHTML = '<h3>Message affiché après 5 clics</h3>';
document.body.appendChild(output);

const button = document.createElement('button');
button.textContent = 'Cliquer';
document.body.appendChild(button);

const counter = document.createElement('div');
counter.style.marginTop = '10px';
counter.textContent = 'Veuillez cliquer 5 fois';
output.appendChild(counter);

const result = document.createElement('div');
result.style.marginTop = '10px';
result.style.fontSize = '18px';
result.style.fontWeight = 'bold';
output.appendChild(result);

let clickCount = 0;

// Événements de clic
const clicks$ = fromEvent(button, 'click');

// Affichage du compteur
clicks$.subscribe(() => {
  clickCount++;
  const remaining = 5 - clickCount;
  if (remaining > 0) {
    counter.textContent = `Encore ${remaining} clics`;
  } else {
    counter.textContent = '';
  }
});

// Détecter le 5ème clic (index 4)
clicks$.pipe(
  elementAt(4)
).subscribe(() => {
  result.textContent = '🎉 Accompli !';
  result.style.color = 'green';
  button.disabled = true;
});
```

- Terminé au 5ème clic (index 4).
- Comme les indices de tableau, il commence à 0.

## 🎯 Exemple de code pratique 2 : Récupérer le N-ième élément d'un flux de données

Un exemple de récupération d'une valeur à un ordre spécifique à partir de données émises à intervalle régulier.

```ts
import { interval } from 'rxjs';
import { elementAt, map } from 'rxjs';

// Création de l'UI
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'Récupérer le N-ième élément du flux de données';
container.appendChild(title);

const input = document.createElement('input');
input.type = 'number';
input.placeholder = 'Entrer un index (0-9)';
input.min = '0';
input.max = '9';
input.style.marginRight = '10px';
container.appendChild(input);

const getButton = document.createElement('button');
getButton.textContent = 'Récupérer';
container.appendChild(getButton);

const status = document.createElement('div');
status.style.marginTop = '10px';
container.appendChild(status);

const result = document.createElement('div');
result.style.marginTop = '10px';
result.style.padding = '10px';
result.style.border = '1px solid #ccc';
result.style.display = 'none';
container.appendChild(result);

// Flux de données (émet toutes les 0.5 secondes, jusqu'à 10 éléments)
const data$ = interval(500).pipe(
  map(i => ({ index: i, value: Math.floor(Math.random() * 100), timestamp: Date.now() }))
);

getButton.addEventListener('click', () => {
  const index = parseInt(input.value);

  if (isNaN(index) || index < 0 || index > 9) {
    status.textContent = 'Veuillez entrer une valeur entre 0 et 9';
    status.style.color = 'red';
    return;
  }

  status.textContent = `Récupération de la valeur à l'index ${index}...`;
  status.style.color = 'blue';
  result.style.display = 'none';
  getButton.disabled = true;
  input.disabled = true;

  data$.pipe(
    elementAt(index)
  ).subscribe({
    next: data => {
      status.textContent = '';
      result.style.display = 'block';
      result.innerHTML = `
        <strong>✅ Récupération réussie</strong><br>
        Index: ${data.index}<br>
        Valeur: ${data.value}<br>
        Horodatage: ${new Date(data.timestamp).toLocaleTimeString()}
      `;
      result.style.color = 'green';
      result.style.backgroundColor = '#e8f5e9';
      getButton.disabled = false;
      input.disabled = false;
    },
    error: err => {
      status.textContent = '';
      result.style.display = 'block';
      result.textContent = `❌ Erreur: ${err.message}`;
      result.style.color = 'red';
      result.style.backgroundColor = '#ffebee';
      getButton.disabled = false;
      input.disabled = false;
    }
  });
});
```

- Récupère la valeur à l'index spécifié d'un flux émettant toutes les 0.5 secondes.
- Une erreur se produit si l'index est hors limites.

## 🆚 Comparaison avec des opérateurs similaires

### elementAt vs take vs first

```ts
import { from } from 'rxjs';
import { elementAt, take, first, skip } from 'rxjs';

const numbers$ = from([10, 20, 30, 40, 50]);

// elementAt: récupère uniquement la valeur à un index spécifique
numbers$.pipe(
  elementAt(2)
).subscribe(console.log);
// Sortie: 30

// take: récupère les N premiers
numbers$.pipe(
  take(3)
).subscribe(console.log);
// Sortie: 10, 20, 30

// skip + first: équivalent à elementAt (redondant)
numbers$.pipe(
  skip(2),
  first()
).subscribe(console.log);
// Sortie: 30
```

| Opérateur | Valeur récupérée | Nombre de sorties | Cas d'utilisation |
|:---|:---|:---|:---|
| `elementAt(n)` | Uniquement la valeur à l'index n | 1 | Récupérer la N-ième valeur |
| `take(n)` | Les n premiers | n | Récupérer les N premiers |
| `first()` | La première valeur | 1 | Récupérer le premier |
| `skip(n) + first()` | Le premier après avoir ignoré n | 1 | Équivalent à elementAt (non recommandé) |

## ⚠️ Points d'attention

### 1. Quand l'index est hors limites

Une erreur se produit si l'index spécifié n'est pas atteint avant que le flux ne se termine.

```ts
import { from } from 'rxjs';
import { elementAt } from 'rxjs';

const numbers$ = from([10, 20, 30]); // Seulement 3 éléments

numbers$.pipe(
  elementAt(5) // Demande l'index 5
).subscribe({
  next: console.log,
  error: err => console.error('Erreur:', err.message)
});
// Sortie: Erreur: no elements in sequence
```

### 2. Spécifier une valeur par défaut

Vous pouvez spécifier une valeur par défaut pour éviter les erreurs.

```ts
import { from } from 'rxjs';
import { elementAt } from 'rxjs';

const numbers$ = from([10, 20, 30]);

// Spécifier une valeur par défaut
numbers$.pipe(
  elementAt(5, 999) // Retourne 999 si l'index 5 n'existe pas
).subscribe({
  next: console.log,
  error: err => console.error('Erreur:', err.message)
});
// Sortie: 999
```

### 3. Utilisation avec des flux asynchrones

Pour les flux asynchrones, il attend jusqu'à ce que la position d'index soit atteinte.

```ts
import { interval } from 'rxjs';
import { elementAt } from 'rxjs';

// Émet une valeur chaque seconde
interval(1000).pipe(
  elementAt(3) // Index 3 (4ème valeur)
).subscribe(console.log);
// Sortie après 3 secondes: 3
```

### 4. Les indices négatifs ne sont pas utilisables

Les indices négatifs ne peuvent pas être spécifiés.

```ts
import { from } from 'rxjs';
import { elementAt } from 'rxjs';

const numbers$ = from([10, 20, 30, 40, 50]);

// ❌ Les indices négatifs provoquent une erreur
numbers$.pipe(
  elementAt(-1)
).subscribe({
  next: console.log,
  error: err => console.error('Erreur:', err.message)
});
// Erreur: ArgumentOutOfRangeError: index out of range
```

Pour récupérer depuis la fin du tableau, utilisez `takeLast` ou `last`.

```ts
import { from } from 'rxjs';
import { takeLast, last } from 'rxjs';

const numbers$ = from([10, 20, 30, 40, 50]);

// ✅ Récupérer la dernière valeur
numbers$.pipe(
  last()
).subscribe(console.log);
// Sortie: 50

// ✅ Récupérer les N derniers
numbers$.pipe(
  takeLast(2)
).subscribe(console.log);
// Sortie: 40, 50
```

## 📚 Opérateurs associés

- **[take](./take)** - Récupérer les N premiers
- **[first](./first)** - Récupérer la première valeur
- **[last](./last)** - Récupérer la dernière valeur
- **[skip](./skip)** - Ignorer les N premiers
- **[takeLast](./takeLast)** - Récupérer les N derniers

## Résumé

L'opérateur `elementAt` récupère uniquement la valeur à la position d'index spécifiée.

- ✅ Même comportement que l'accès par index des tableaux
- ✅ Optimal pour récupérer la N-ième valeur
- ✅ Peut spécifier une valeur par défaut pour éviter les erreurs
- ⚠️ Erreur si l'index est hors limites (sans valeur par défaut)
- ⚠️ Les indices négatifs ne sont pas utilisables
- ⚠️ Attend jusqu'à ce que la position soit atteinte pour les flux asynchrones

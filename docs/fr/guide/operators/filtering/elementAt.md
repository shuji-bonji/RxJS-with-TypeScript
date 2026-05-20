---
description: "L'opérateur elementAt est un opérateur de filtrage de RxJS qui ne récupère que les valeurs à une position d'index donnée. Son fonctionnement est similaire à celui de l'accès à l'index d'un tableau."
---

# elementAt - Récupéré par spécification d'index

L'opérateur `elementAt` récupère **uniquement la valeur à la position d'index spécifiée** de l'Observable et termine le flux immédiatement. Il fonctionne de la même manière que `array[index]` d'un tableau.

## 🔰 Syntaxe de base et utilisation

```ts
import { from } from 'rxjs';
import { elementAt } from 'rxjs';

const numbers$ = from([10, 20, 30, 40, 50]);

numbers$.pipe(
  elementAt(2)
).subscribe(console.log);
// Sortie.: 30(Valeur2Valeur)
```

**Flux d'opérations** :.
1. 10 (index 0) → sauter
2. 20 (index 1) → saut
3. 30 (index 2) → sortie et terminé
4. 40, 50 non évalués

[🌐 Official RxJS documentation - `elementAt`](https://rxjs.dev/api/operators/elementAt)

## 💡 Modèle d'utilisation typique.

- **Pagination** : obtenir le premier élément d'une page spécifique.
- Obtenir des données garanties par l'ordre** : obtenir le Nième événement ou message.
- **Test et débogage** : valider la valeur d'une position spécifique.
- **Accès de type tableau** : traiter Observable comme un tableau.

## 🧠 Exemple de code pratique 1 : Compte à rebours d'événements

Voici un exemple d'exécution d'une action au Nième clic.

```ts
import { fromEvent } from 'rxjs';
import { elementAt, map } from 'rxjs';

// UICréer
const output = document.createElement('div');
output.innerHTML = '<h3>5Cliquer une fois pour afficher le message</h3>';
document.body.appendChild(output);

const button = document.createElement('button');
button.textContent = 'Cliquer';
document.body.appendChild(button);

const counter = document.createElement('div');
counter.style.marginTop = '10px';
counter.textContent = 'plus5Cliquer une fois';
output.appendChild(counter);

const result = document.createElement('div');
result.style.marginTop = '10px';
result.style.fontSize = '18px';
result.style.fontWeight = 'bold';
output.appendChild(result);

let clickCount = 0;

// Cliquer sur un événement
const clicks$ = fromEvent(button, 'click');

// Pour l'affichage du compte
clicks$.subscribe(() => {
  clickCount++;
  const remaining = 5 - clickCount;
  if (remaining > 0) {
    counter.textContent = `plus${remaining}Cliquer une fois`;
  } else {
    counter.textContent = '';
  }
});

// 5Deuxième fois (index)4Clics détectés de
clicks$.pipe(
  elementAt(4)
).subscribe(() => {
  result.textContent = '🎉 Atteint！';
  result.style.color = 'green';
  button.disabled = true;
});
```

- Le cinquième clic (index 4) termine l'action.
- Il commence à 0, tout comme l'index du tableau.

## 🎯 Exemple de code pratique 2 : Obtenir le Nième nombre du flux de données.

Voici un exemple de récupération d'un ordre spécifique de valeurs à partir de données publiées à intervalles réguliers.

```ts
import { interval } from 'rxjs';
import { elementAt, map } from 'rxjs';

// UICréer
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'A partir du flux de donnéesNObtenir le deuxième';
container.appendChild(title);

const input = document.createElement('input');
input.type = 'number';
input.placeholder = 'Saisir l'indice (0〜du flux de données (9)';
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

// Flux de données (0.5Les valeurs sont émises toutes les secondes,10jusqu'à 1)
const data$ = interval(500).pipe(
  map(i => ({ index: i, value: Math.floor(Math.random() * 100), timestamp: Date.now() }))
);

getButton.addEventListener('click', () => {
  const index = parseInt(input.value);

  if (isNaN(index) || index < 0 || index > 9) {
    status.textContent = '0〜du flux de données (9Veuillez saisir une plage de';
    status.style.color = 'red';
    return;
  }

  status.textContent = `Index ${index} La valeur est en cours d'extraction...`;
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

- Récupère les valeurs à un index spécifié à partir d'un flux publié toutes les 0,5 secondes.
- Une erreur est générée si l'index est hors de portée.

## 🆚 Comparaison avec des opérateurs similaires

### elementAt vs take vs first

```ts
import { from } from 'rxjs';
import { elementAt, take, first, skip } from 'rxjs';

const numbers$ = from([10, 20, 30, 40, 50]);

// elementAt: Récupérer uniquement les valeurs d'un index spécifique
numbers$.pipe(
  elementAt(2)
).subscribe(console.log);
// Sortie.: 30

// take: Depuis le débutNObtenir une seule valeur
numbers$.pipe(
  take(3)
).subscribe(console.log);
// Sortie.: 10, 20, 30

// skip + first: elementAt Équivalent à (redondant)
numbers$.pipe(
  skip(2),
  first()
).subscribe(console.log);
// Sortie.: 30
```

```ts
import { from } from 'rxjs';
import { elementAt } from 'rxjs';

const numbers$ = from([10, 20, 30, 40, 50]);

numbers$.pipe(
  elementAt(2)
).subscribe(console.log);
// Sortie.: 30(Valeur2Valeur)
```

## ⚠️ Notes.

### 1. Si l'index est hors plage

Si l'index spécifié n'est pas atteint avant la fin du flux, une erreur est générée.


```ts
import { from } from 'rxjs';
import { elementAt } from 'rxjs';

const numbers$ = from([10, 20, 30]); // 3Une seule

numbers$.pipe(
  elementAt(5) // Index5Demander
).subscribe({
  next: console.log,
  error: err => console.error('Erreur:', err.message)
});
// Sortie.: Erreur: no elements in sequence
```

### Spécifier les valeurs par défaut.

Pour éviter les erreurs, des valeurs par défaut peuvent être spécifiées.

```ts
import { from } from 'rxjs';
import { elementAt } from 'rxjs';

const numbers$ = from([10, 20, 30]);

// Spécifier une valeur par défaut
numbers$.pipe(
  elementAt(5, 999) // Index5S'il n'y en a pas, renvoie999Renvoie une
).subscribe({
  next: console.log,
  error: err => console.error('Erreur:', err.message)
});
// Sortie.: 999
```

### Utilisation avec des flux asynchrones

Dans les flux asynchrones, attendez que la position de l'index soit atteinte.

```ts
import { interval } from 'rxjs';
import { elementAt } from 'rxjs';

// 1Délivre une valeur toutes les secondes
interval(1000).pipe(
  elementAt(3) // Index3(4(valeur de la seconde)
).subscribe(console.log);
// 3Sortie après quelques secondes: 3
```

### Les index négatifs ne sont pas autorisés.

Les index négatifs ne peuvent pas être spécifiés.

```ts
import { from } from 'rxjs';
import { elementAt } from 'rxjs';

const numbers$ = from([10, 20, 30, 40, 50]);

// ❌ Les index négatifs sont des erreurs
numbers$.pipe(
  elementAt(-1)
).subscribe({
  next: console.log,
  error: err => console.error('Erreur:', err.message)
});
// Erreur: ArgumentOutOfRangeError: index out of range
```

Utilisez `takeLast` ou `last` pour aller à la fin du tableau.

```ts
import { from } from 'rxjs';
import { takeLast, last } from 'rxjs';

const numbers$ = from([10, 20, 30, 40, 50]);

// ✅ Obtenir la dernière valeur
numbers$.pipe(
  last()
).subscribe(console.log);
// Sortie.: 50

// ✅ Obtenir la dernière valeurNObtenir la dernière valeur
numbers$.pipe(
  takeLast(2)
).subscribe(console.log);
// Sortie.: 40, 50
```

## 📚 Opérateurs apparentés.

- **[take](./take)** - N pris depuis le début.
- **[first](./first)** - obtient la première valeur.
- **[last](./last)** - obtient la dernière valeur.
- **[skip](./skip)** - sauter les N premières valeurs
- **[takeLast](./takeLast)** - obtenir les N dernières valeurs

## Résumé.

L'opérateur `elementAt` ne récupère que la valeur à la position d'index spécifiée.

- ✅ Même comportement que l'accès à l'index d'un tableau.
- ✅ Idéal pour récupérer la Nième valeur
- ✅ Les valeurs par défaut peuvent être spécifiées pour éviter les erreurs.
- ⚠️ Erreur si l'index est en dehors de la plage (pas de valeur par défaut)
- ⚠️ Les index négatifs ne sont pas autorisés
- ⚠️ Les flux asynchrones attendent d'être atteints

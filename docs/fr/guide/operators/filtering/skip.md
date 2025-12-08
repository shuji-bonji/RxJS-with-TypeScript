---
description: "L'opérateur skip ignore le premier nombre spécifié de valeurs d'un flux Observable et n'émet que les valeurs suivantes. Utile lorsque vous souhaitez ignorer les données initiales ou sauter une période d'échauffement."
---

# skip - Ignorer les N premières valeurs

L'opérateur `skip` ignore **le premier nombre spécifié** de valeurs d'un flux et n'émet que les valeurs suivantes.


## 🔰 Syntaxe de base et utilisation

```ts
import { interval } from 'rxjs';
import { skip } from 'rxjs';

const source$ = interval(1000);

source$.pipe(
  skip(3)
).subscribe(console.log);
// Sortie: 3, 4, 5, 6, 7, ...
```

- Les 3 premières valeurs (0, 1, 2) sont ignorées
- Toutes les valeurs à partir de la 4ème (3, 4, 5, ...) sont émises
- Le flux se termine au moment de fin du flux original

[🌐 Documentation officielle RxJS - `skip`](https://rxjs.dev/api/operators/skip)


## 🆚 Comparaison avec take

`skip` et `take` ont des comportements opposés.

```ts
import { range } from 'rxjs';
import { skip, take } from 'rxjs';

const numbers$ = range(0, 10); // 0 à 9

// take: récupère les N premiers
numbers$.pipe(
  take(3)
).subscribe(console.log);
// Sortie: 0, 1, 2

// skip: ignore les N premiers
numbers$.pipe(
  skip(3)
).subscribe(console.log);
// Sortie: 3, 4, 5, 6, 7, 8, 9

// Combinaison: ignorer les 3 premiers et récupérer les 3 suivants
numbers$.pipe(
  skip(3),
  take(3)
).subscribe(console.log);
// Sortie: 3, 4, 5
```

| Opérateur | Comportement | Moment de fin |
|---|---|---|
| `take(n)` | Récupère les n premiers | Fin automatique après n éléments |
| `skip(n)` | Ignore les n premiers | À la fin du flux original |


## 💡 Patterns d'utilisation typiques

1. **Ignorer la valeur initiale**
   ```ts
   import { BehaviorSubject } from 'rxjs';
   import { skip } from 'rxjs';

   const state$ = new BehaviorSubject<number>(0);

   // Ignorer la valeur initiale et surveiller uniquement les changements
   state$.pipe(
     skip(1)
   ).subscribe(value => {
     console.log(`L'état a changé: ${value}`);
   });

   state$.next(1); // Sortie: L'état a changé: 1
   state$.next(2); // Sortie: L'état a changé: 2
   ```

2. **Ignorer la période d'échauffement**
   ```ts
   import { interval } from 'rxjs';
   import { skip, map } from 'rxjs';

   // Simulation de données de capteur
   const sensorData$ = interval(100).pipe(
     map(() => Math.random() * 100)
   );

   // Ignorer les 10 premières valeurs (1 seconde) comme période de calibration
   sensorData$.pipe(
     skip(10)
   ).subscribe(data => {
     console.log(`Valeur du capteur: ${data.toFixed(2)}`);
   });
   ```

3. **Pagination**
   ```ts
   import { from } from 'rxjs';
   import { skip, take } from 'rxjs';

   interface Item {
     id: number;
     name: string;
   }

   const allItems$ = from([
     { id: 1, name: 'Item 1' },
     { id: 2, name: 'Item 2' },
     { id: 3, name: 'Item 3' },
     { id: 4, name: 'Item 4' },
     { id: 5, name: 'Item 5' },
     { id: 6, name: 'Item 6' },
   ] as Item[]);

   const pageSize = 2;
   const pageNumber = 2; // indexé à partir de 0

   // Récupérer les éléments de la page 2 (items 5 et 6)
   allItems$.pipe(
     skip(pageNumber * pageSize),
     take(pageSize)
   ).subscribe(item => {
     console.log(item);
   });
   // Sortie: { id: 5, name: 'Item 5' }, { id: 6, name: 'Item 6' }
   ```


## 🧠 Exemple de code pratique (compteur)

Exemple qui ignore les 3 premiers clics et compte uniquement à partir du 4ème.

```ts
import { fromEvent } from 'rxjs';
import { skip, scan } from 'rxjs';

// Création des éléments UI
const container = document.createElement('div');
document.body.appendChild(container);

const button = document.createElement('button');
button.textContent = 'Cliquer';
container.appendChild(button);

const counter = document.createElement('div');
counter.style.marginTop = '10px';
counter.textContent = 'Compteur: 0';
container.appendChild(counter);

const message = document.createElement('div');
message.style.marginTop = '5px';
message.style.color = 'gray';
message.textContent = 'Les 3 premiers clics sont ignorés';
container.appendChild(message);

// Événement de clic
fromEvent(button, 'click').pipe(
  skip(3), // Ignorer les 3 premiers
  scan((count) => count + 1, 0)
).subscribe(count => {
  counter.textContent = `Compteur: ${count}`;
  if (count === 1) {
    message.textContent = 'Comptage commencé à partir du 4ème clic!';
    message.style.color = 'green';
  }
});
```

Ce code ignore les 3 premiers clics et commence à compter à partir de « 1 » au 4ème clic.


## 🎯 Différence entre skip et skipWhile

```ts
import { of } from 'rxjs';
import { skip, skipWhile } from 'rxjs';

const numbers$ = of(1, 2, 3, 4, 5, 6);

// skip: ignore les N premiers par nombre
numbers$.pipe(
  skip(3)
).subscribe(console.log);
// Sortie: 4, 5, 6

// skipWhile: ignore tant que la condition est satisfaite
numbers$.pipe(
  skipWhile(n => n < 4)
).subscribe(console.log);
// Sortie: 4, 5, 6
```

| Opérateur | Condition d'ignorance | Cas d'utilisation |
|---|---|---|
| `skip(n)` | Ignorer les n premiers par nombre | Ignorance d'un nombre fixe |
| `skipWhile(predicate)` | Ignorer tant que la condition est satisfaite | Ignorance basée sur une condition |
| `skipUntil(notifier$)` | Ignorer jusqu'à ce qu'un autre Observable émette | Ignorance basée sur le temps |


## 📋 Utilisation type-safe

Exemple d'implémentation type-safe avec les génériques TypeScript.

```ts
import { Observable, from } from 'rxjs';
import { skip, take } from 'rxjs';

interface User {
  id: number;
  name: string;
  role: 'admin' | 'user';
}

function getPaginatedUsers(
  users$: Observable<User>,
  page: number,
  pageSize: number
): Observable<User> {
  return users$.pipe(
    skip(page * pageSize),
    take(pageSize)
  );
}

// Exemple d'utilisation
const users$ = from([
  { id: 1, name: 'Alice', role: 'admin' as const },
  { id: 2, name: 'Bob', role: 'user' as const },
  { id: 3, name: 'Charlie', role: 'user' as const },
  { id: 4, name: 'Dave', role: 'admin' as const },
  { id: 5, name: 'Eve', role: 'user' as const },
] as User[]);

// Récupérer la page 1 (2ème page, indexée à partir de 0)
getPaginatedUsers(users$, 1, 2).subscribe(user => {
  console.log(`${user.name} (${user.role})`);
});
// Sortie: Charlie (user), Dave (admin)
```


## ⚠️ Erreurs courantes

> [!NOTE]
> `skip` ignore uniquement les N premiers et ne termine pas le flux. Avec les flux infinis, combinez avec `take` pour définir une condition de fin.

### Incorrect: utiliser uniquement skip avec un flux infini

```ts
import { interval } from 'rxjs';
import { skip } from 'rxjs';

// ❌ Mauvais exemple: le flux infini continue indéfiniment
interval(1000).pipe(
  skip(5)
).subscribe(console.log);
// 5, 6, 7, 8, ... continue indéfiniment
```

### Correct: combiner avec take pour définir une condition de fin

```ts
import { interval } from 'rxjs';
import { skip, take } from 'rxjs';

// ✅ Bon exemple: limiter le nombre après avoir ignoré
interval(1000).pipe(
  skip(5),
  take(3)
).subscribe({
  next: console.log,
  complete: () => console.log('Terminé')
});
// 5, 6, 7, Terminé
```


## 🎓 Résumé

### Quand utiliser skip
- ✅ Lorsque vous voulez ignorer les données initiales ou les N premières
- ✅ Lorsque vous voulez ignorer la valeur initiale d'un BehaviorSubject
- ✅ Lorsque vous voulez récupérer les données d'une page spécifique avec la pagination
- ✅ Lorsque vous voulez ignorer la période de calibration d'un capteur

### Combinaison avec take
- ✅ Lorsque vous voulez récupérer uniquement une plage spécifique de données
- ✅ Lorsque vous voulez récupérer la partie intermédiaire d'un flux infini

### Points d'attention
- ⚠️ Avec les flux infinis, combinez avec `take` pour définir une condition de fin
- ⚠️ `skip(0)` a le même comportement que le flux original (n'ignore rien)
- ⚠️ Si le nombre à ignorer est supérieur au nombre total de données, rien n'est émis et le flux se termine


## 🚀 Prochaines étapes

- **[take](./take)** - Apprendre à récupérer les N premières valeurs
- **[first](./first)** - Apprendre à récupérer la première valeur ou la première satisfaisant une condition
- **[last](./last)** - Apprendre à récupérer la dernière valeur
- **[filter](./filter)** - Apprendre le filtrage basé sur les conditions
- **[Exemples pratiques d'opérateurs de filtrage](./practical-use-cases)** - Apprendre des cas d'utilisation réels

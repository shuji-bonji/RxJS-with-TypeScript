---
description: "L'opérateur skipWhile ignore les valeurs tant qu'une condition spécifiée est remplie, et émet toutes les valeurs suivantes une fois que la condition devient fausse. Utile lorsque vous souhaitez contrôler le flux à l'aide d'une condition de démarrage dynamique."
---

# skipWhile - Ignorer Pendant Condition

L'opérateur `skipWhile` **ignore les valeurs tant que la condition spécifiée est satisfaite**, et une fois que la condition devient `false`, **émet toutes les valeurs suivantes**.

## 🔰 Syntaxe de base et utilisation

```ts
import { range } from 'rxjs';
import { skipWhile } from 'rxjs';

const numbers$ = range(0, 10); // 0 à 9

numbers$.pipe(
  skipWhile(n => n < 5)
).subscribe(console.log);
// Sortie: 5, 6, 7, 8, 9
```

**Flux d'opération** :
1. 0 est émis → `0 < 5` est `true` → ignoré
2. 1 est émis → `1 < 5` est `true` → ignoré
3. 2 est émis → `2 < 5` est `true` → ignoré
4. 3 est émis → `3 < 5` est `true` → ignoré
5. 4 est émis → `4 < 5` est `true` → ignoré
6. 5 est émis → `5 < 5` est `false` → début de l'émission
7. 6 et suivants → tout est émis (la condition n'est plus réévaluée)

[🌐 Documentation officielle RxJS - `skipWhile`](https://rxjs.dev/api/operators/skipWhile)

## 💡 Patterns d'utilisation typiques

- **Ignorer les données initiales inutiles** : exclure les données de la période d'échauffement
- **Ignorer jusqu'à un seuil** : attendre qu'une condition spécifique soit satisfaite
- **Ignorer les lignes d'en-tête** : exclure les en-têtes CSV, etc.
- **Ignorer la période de préparation** : attendre que le système soit prêt

## 🧠 Exemple de code pratique 1 : Ignorer la période d'échauffement du capteur

Exemple d'ignorance des données initiales jusqu'à ce que le capteur se stabilise.

```ts
import { interval } from 'rxjs';
import { skipWhile, map, take } from 'rxjs';

// Création de l'UI
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'Surveillance du capteur de température';
container.appendChild(title);

const status = document.createElement('div');
status.style.marginBottom = '10px';
status.style.padding = '10px';
status.style.backgroundColor = '#fff3e0';
status.style.border = '1px solid #FF9800';
status.textContent = '🔄 Préparation du capteur... (mesure commence à 20°C ou plus)';
container.appendChild(status);

const output = document.createElement('div');
output.style.border = '1px solid #ccc';
output.style.padding = '10px';
output.style.maxHeight = '200px';
output.style.overflow = 'auto';
container.appendChild(output);

let isWarmedUp = false;

// Simulation du capteur de température (se réchauffe progressivement)
interval(500).pipe(
  take(20),
  map(i => {
    // Basse température au début, augmente progressivement
    const baseTemp = 15 + i * 0.5;
    const noise = (Math.random() - 0.5) * 2;
    return baseTemp + noise;
  }),
  skipWhile(temp => temp < 20) // Ignorer en dessous de 20°C
).subscribe({
  next: temp => {
    // Mettre à jour le statut à la première valeur
    if (!isWarmedUp) {
      isWarmedUp = true;
      status.textContent = '✅ Capteur prêt (mesure commencée)';
      status.style.backgroundColor = '#e8f5e9';
      status.style.borderColor = '#4CAF50';
    }

    const log = document.createElement('div');
    log.style.padding = '5px';
    log.style.marginBottom = '3px';
    log.style.backgroundColor = temp > 25 ? '#ffebee' : '#f1f8e9';
    log.textContent = `[${new Date().toLocaleTimeString()}] Température: ${temp.toFixed(1)}°C`;
    output.insertBefore(log, output.firstChild);

    // Afficher maximum 10 entrées
    while (output.children.length > 10) {
      output.removeChild(output.lastChild!);
    }
  },
  complete: () => {
    const summary = document.createElement('div');
    summary.style.marginTop = '10px';
    summary.style.padding = '10px';
    summary.style.backgroundColor = '#e3f2fd';
    summary.textContent = 'Mesure terminée';
    container.appendChild(summary);
  }
});
```

- Les données sont ignorées tant que le capteur est en dessous de 20°C.
- Toutes les données sont enregistrées à partir de 20°C et plus.

## 🎯 Exemple de code pratique 2 : Traitement des événements après préparation

Exemple d'ignorance des événements jusqu'à ce que l'initialisation du système soit terminée.

```ts
import { fromEvent, merge, Subject } from 'rxjs';
import { skipWhile, map, tap } from 'rxjs';

// Création de l'UI
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'Système de traitement des événements';
container.appendChild(title);

const initButton = document.createElement('button');
initButton.textContent = 'Initialisation terminée';
initButton.style.marginRight = '10px';
container.appendChild(initButton);

const eventButton = document.createElement('button');
eventButton.textContent = 'Déclencher événement';
container.appendChild(eventButton);

const statusDiv = document.createElement('div');
statusDiv.style.marginTop = '10px';
statusDiv.style.padding = '10px';
statusDiv.style.backgroundColor = '#ffebee';
statusDiv.style.border = '1px solid #f44336';
statusDiv.innerHTML = '<strong>⏸️ Système non initialisé</strong><br>Les événements sont ignorés';
container.appendChild(statusDiv);

const eventLog = document.createElement('div');
eventLog.style.marginTop = '10px';
eventLog.style.border = '1px solid #ccc';
eventLog.style.padding = '10px';
eventLog.style.minHeight = '100px';
container.appendChild(eventLog);

// État d'initialisation
let isInitialized = false;
const initSubject = new Subject<boolean>();

// Bouton d'initialisation
fromEvent(initButton, 'click').subscribe(() => {
  if (!isInitialized) {
    isInitialized = true;
    initSubject.next(true);
    statusDiv.style.backgroundColor = '#e8f5e9';
    statusDiv.style.borderColor = '#4CAF50';
    statusDiv.innerHTML = '<strong>✅ Système initialisé</strong><br>Les événements sont traités';
    initButton.disabled = true;
  }
});

// Traitement des événements (ignoré jusqu'à l'initialisation)
let eventCount = 0;
fromEvent(eventButton, 'click').pipe(
  map(() => {
    eventCount++;
    return {
      id: eventCount,
      timestamp: new Date(),
      initialized: isInitialized
    };
  }),
  tap(event => {
    if (!event.initialized) {
      const skipLog = document.createElement('div');
      skipLog.style.padding = '5px';
      skipLog.style.marginBottom = '3px';
      skipLog.style.color = '#999';
      skipLog.textContent = `⏭️ Événement #${event.id} ignoré (non initialisé)`;
      eventLog.insertBefore(skipLog, eventLog.firstChild);
    }
  }),
  skipWhile(event => !event.initialized)
).subscribe(event => {
  const log = document.createElement('div');
  log.style.padding = '5px';
  log.style.marginBottom = '3px';
  log.style.backgroundColor = '#e8f5e9';
  log.style.border = '1px solid #4CAF50';
  log.innerHTML = `
    <strong>✅ Événement #${event.id} traité</strong>
    [${event.timestamp.toLocaleTimeString()}]
  `;
  eventLog.insertBefore(log, eventLog.firstChild);

  // Afficher maximum 10 entrées
  while (eventLog.children.length > 10) {
    eventLog.removeChild(eventLog.lastChild!);
  }
});
```

- Tous les événements sont ignorés jusqu'à ce que le système soit initialisé.
- Après l'initialisation, tous les événements sont traités.

## 🆚 Comparaison avec des opérateurs similaires

### skipWhile vs takeWhile vs skip vs filter

```ts
import { range } from 'rxjs';
import { skipWhile, takeWhile, skip, filter } from 'rxjs';

const numbers$ = range(0, 10); // 0 à 9

// skipWhile: ignorer tant que la condition est satisfaite, émettre tout après
numbers$.pipe(
  skipWhile(n => n < 5)
).subscribe(console.log);
// Sortie: 5, 6, 7, 8, 9

// takeWhile: récupérer uniquement tant que la condition est satisfaite
numbers$.pipe(
  takeWhile(n => n < 5)
).subscribe(console.log);
// Sortie: 0, 1, 2, 3, 4

// skip: ignorer les N premiers
numbers$.pipe(
  skip(5)
).subscribe(console.log);
// Sortie: 5, 6, 7, 8, 9

// filter: seules les valeurs satisfaisant la condition passent (évaluation complète)
numbers$.pipe(
  filter(n => n >= 5)
).subscribe(console.log);
// Sortie: 5, 6, 7, 8, 9
```

| Opérateur | Comportement | Réévaluation de la condition | Moment de fin |
|:---|:---|:---|:---|
| `skipWhile(predicate)` | Ignorer tant que la condition est satisfaite | Non (terminé une fois false) | Fin du flux original |
| `takeWhile(predicate)` | Récupérer tant que la condition est satisfaite | À chaque valeur | Quand la condition devient false |
| `skip(n)` | Ignorer les n premiers | Non (basé sur le nombre) | Fin du flux original |
| `filter(predicate)` | Seules les valeurs satisfaisantes passent | **À chaque valeur** | Fin du flux original |

**Différence visuelle** :

```
Entrée: 0, 1, 2, 3, 4, 5, 4, 3, 2, 1, 0

skipWhile(n => n < 5):
[0,1,2,3,4 ignorés] | 5, 4, 3, 2, 1, 0
                      ^tout émis après que la condition devient false

filter(n => n >= 5):
[0,1,2,3,4 exclus] 5 [4,3,2,1,0 exclus]
                   ^seules les valeurs satisfaisantes (évaluation à chaque fois)

takeWhile(n => n < 5):
0, 1, 2, 3, 4 | [5 et suivants ignorés, terminé]
```

## ⚠️ Points d'attention

### 1. La condition n'est plus réévaluée une fois false

C'est la plus grande différence avec `filter`.

```ts
import { from } from 'rxjs';
import { skipWhile, filter } from 'rxjs';

const numbers$ = from([1, 2, 3, 4, 5, 4, 3, 2, 1]);

// skipWhile: une fois la condition false, tout est émis après
numbers$.pipe(
  skipWhile(n => n < 5)
).subscribe(val => console.log('skipWhile:', val));
// Sortie: skipWhile: 5, 4, 3, 2, 1 (tout après 5)

// filter: évaluation de la condition à chaque fois
numbers$.pipe(
  filter(n => n >= 5)
).subscribe(val => console.log('filter:', val));
// Sortie: filter: 5 (uniquement 5)
```

### 2. Si la condition est false dès le départ

Si la condition est `false` dès le départ, toutes les valeurs sont émises.

```ts
import { range } from 'rxjs';
import { skipWhile } from 'rxjs';

range(5, 5).pipe( // 5 à 9
  skipWhile(n => n < 3) // La condition est false dès le départ
).subscribe(console.log);
// Sortie: 5, 6, 7, 8, 9 (tout émis)
```

### 3. Si toutes les valeurs satisfont la condition

Si toutes les valeurs satisfont la condition, rien n'est émis.

```ts
import { range } from 'rxjs';
import { skipWhile } from 'rxjs';

range(0, 5).pipe( // 0 à 4
  skipWhile(n => n < 10) // Toutes les valeurs satisfont la condition
).subscribe({
  next: console.log,
  complete: () => console.log('Terminé (rien émis)')
});
// Sortie: Terminé (rien émis)
```

### 4. Types TypeScript

`skipWhile` ne change pas le type.

```ts
import { Observable, from } from 'rxjs';
import { skipWhile } from 'rxjs';

interface User {
  id: number;
  name: string;
  isActive: boolean;
}

const users$: Observable<User> = from([
  { id: 1, name: 'Alice', isActive: false },
  { id: 2, name: 'Bob', isActive: false },
  { id: 3, name: 'Charlie', isActive: true },
  { id: 4, name: 'Dave', isActive: true }
]);

// Le type reste Observable<User>
const activeUsers$: Observable<User> = users$.pipe(
  skipWhile(user => !user.isActive)
);

activeUsers$.subscribe(user => {
  console.log(`${user.name} (ID: ${user.id})`);
});
// Sortie: Charlie (ID: 3), Dave (ID: 4)
```

## 💡 Patterns de combinaison pratiques

### Pattern 1 : Ignorer les lignes d'en-tête

Ignorer les lignes d'en-tête CSV, etc.

```ts
import { from } from 'rxjs';
import { skipWhile, map } from 'rxjs';

const csvLines$ = from([
  'Name,Age,City',     // Ligne d'en-tête
  'Alice,25,Tokyo',
  'Bob,30,Osaka',
  'Charlie,35,Kyoto'
]);

let isFirstLine = true;

csvLines$.pipe(
  skipWhile(() => {
    if (isFirstLine) {
      isFirstLine = false;
      return true; // Ignorer la première ligne (en-tête)
    }
    return false;
  }),
  map(line => {
    const [name, age, city] = line.split(',');
    return { name, age: Number(age), city };
  })
).subscribe(console.log);
// Sortie:
// { name: 'Alice', age: 25, city: 'Tokyo' }
// { name: 'Bob', age: 30, city: 'Osaka' }
// { name: 'Charlie', age: 35, city: 'Kyoto' }
```

### Pattern 2 : Filtrage basé sur l'horodatage

Traiter uniquement les données après une heure spécifique

```ts
import { from } from 'rxjs';
import { skipWhile } from 'rxjs';

interface LogEntry {
  timestamp: Date;
  message: string;
}

const startTime = new Date('2025-01-01T12:00:00');

const logs$ = from([
  { timestamp: new Date('2025-01-01T10:00:00'), message: 'Log 1' },
  { timestamp: new Date('2025-01-01T11:00:00'), message: 'Log 2' },
  { timestamp: new Date('2025-01-01T12:00:00'), message: 'Log 3' },
  { timestamp: new Date('2025-01-01T13:00:00'), message: 'Log 4' }
] as LogEntry[]);

logs$.pipe(
  skipWhile(log => log.timestamp < startTime)
).subscribe(log => {
  console.log(`[${log.timestamp.toISOString()}] ${log.message}`);
});
// Sortie:
// [2025-01-01T12:00:00.000Z] Log 3
// [2025-01-01T13:00:00.000Z] Log 4
```

### Pattern 3 : Ignorance basée sur l'état

Ignorer jusqu'à ce que le système soit prêt

```ts
import { interval } from 'rxjs';
import { skipWhile, map, take } from 'rxjs';

interface SystemState {
  tick: number;
  isReady: boolean;
  data: number;
}

// Simulation de l'état du système
interval(500).pipe(
  take(10),
  map(i => ({
    tick: i,
    isReady: i >= 3, // Prêt après 3 secondes
    data: Math.floor(Math.random() * 100)
  } as SystemState)),
  skipWhile(state => !state.isReady)
).subscribe(state => {
  console.log(`Tick ${state.tick}: données=${state.data}`);
});
// Sortie: uniquement les données à partir du tick 3
```

## 📚 Opérateurs associés

- **[takeWhile](./takeWhile)** - Récupérer uniquement tant que la condition est satisfaite
- **[skip](./skip)** - Ignorer les N premières valeurs
- **[skipLast](./skipLast)** - Ignorer les N dernières valeurs
- **[skipUntil](./skipUntil)** - Ignorer jusqu'à ce qu'un autre Observable émette
- **[filter](./filter)** - Seules les valeurs satisfaisantes passent

## Résumé

L'opérateur `skipWhile` ignore les valeurs tant que la condition est satisfaite, et émet toutes les valeurs suivantes une fois que la condition devient false.

- ✅ Idéal pour ignorer les données initiales inutiles
- ✅ La condition n'est plus réévaluée une fois false
- ✅ Pratique pour ignorer la période d'échauffement ou de préparation
- ✅ Utilisable pour ignorer les lignes d'en-tête
- ⚠️ Contrairement à `filter`, la condition n'est évaluée qu'une fois
- ⚠️ Si toutes les valeurs satisfont la condition, rien n'est émis
- ⚠️ Continue jusqu'à la fin du flux original

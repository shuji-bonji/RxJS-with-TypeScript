---
description: "takeWhile est un opérateur de filtrage RxJS qui continue à récupérer des valeurs tant qu'une condition spécifiée est remplie, et termine le flux lorsque la condition devient fausse. Idéal pour l'extraction de données jusqu'à un seuil, le traitement basé sur la priorité et la pagination lorsque vous souhaitez contrôler le flux avec des conditions dynamiques. L'option inclusive permet d'inclure la valeur qui a rendu la condition fausse."
---

# takeWhile - Prendre Pendant Condition

L'opérateur `takeWhile` continue à récupérer des valeurs **tant que la condition spécifiée est satisfaite**, et termine le flux lorsque la condition devient `false`.


## 🔰 Syntaxe de base et utilisation

```ts
import { interval } from 'rxjs';
import { takeWhile } from 'rxjs';

const source$ = interval(1000);

source$.pipe(
  takeWhile(n => n < 5)
).subscribe({
  next: console.log,
  complete: () => console.log('Terminé')
});
// Sortie: 0, 1, 2, 3, 4, Terminé
```

**Flux d'opération** :
1. 0 est émis → `0 < 5` est `true` → sortie
2. 1 est émis → `1 < 5` est `true` → sortie
3. 2 est émis → `2 < 5` est `true` → sortie
4. 3 est émis → `3 < 5` est `true` → sortie
5. 4 est émis → `4 < 5` est `true` → sortie
6. 5 est émis → `5 < 5` est `false` → terminé (5 n'est pas émis)

[🌐 Documentation officielle RxJS - `takeWhile`](https://rxjs.dev/api/operators/takeWhile)


## 🆚 Comparaison avec take

`take` et `takeWhile` ont des conditions de récupération différentes.

```ts
import { interval } from 'rxjs';
import { take, takeWhile } from 'rxjs';

const source$ = interval(1000);

// take: contrôle par nombre
source$.pipe(
  take(5)
).subscribe(console.log);
// Sortie: 0, 1, 2, 3, 4

// takeWhile: contrôle par condition
source$.pipe(
  takeWhile(n => n < 5)
).subscribe(console.log);
// Sortie: 0, 1, 2, 3, 4
```

| Opérateur | Méthode de contrôle | Condition de fin | Dernière valeur |
|---|---|---|---|
| `take(n)` | Nombre | Après n éléments | Inclut le n-ième |
| `takeWhile(predicate)` | Fonction de condition | Quand la condition devient `false` | N'inclut pas la valeur `false`* |

\* Par défaut, la valeur qui rend la condition `false` n'est pas émise, mais peut être incluse avec `inclusive: true`


## 🎯 Option inclusive

Pour inclure la valeur qui a rendu la condition `false`, spécifiez `inclusive: true`.

```ts
import { range } from 'rxjs';
import { takeWhile } from 'rxjs';

const numbers$ = range(0, 10);

// Par défaut (inclusive: false)
numbers$.pipe(
  takeWhile(n => n < 5)
).subscribe(console.log);
// Sortie: 0, 1, 2, 3, 4

// inclusive: true
numbers$.pipe(
  takeWhile(n => n < 5, true)
).subscribe(console.log);
// Sortie: 0, 1, 2, 3, 4, 5 (inclut 5 qui a rendu la condition false)
```


## 💡 Patterns d'utilisation typiques

1. **Récupération de données jusqu'à un seuil**
   ```ts
   import { interval } from 'rxjs';
   import { takeWhile, map } from 'rxjs';

   // Simulation de capteur de température
   const temperature$ = interval(100).pipe(
     map(() => 20 + Math.random() * 15)
   );

   // Enregistrer uniquement tant que la température est inférieure à 30°C
   temperature$.pipe(
     takeWhile(temp => temp < 30)
   ).subscribe({
     next: temp => console.log(`Température: ${temp.toFixed(1)}°C`),
     complete: () => console.log('Alerte: La température a dépassé 30°C!')
   });
   ```

2. **Traitement conditionnel de tableaux**
   ```ts
   import { from } from 'rxjs';
   import { takeWhile } from 'rxjs';

   interface Task {
     id: number;
     priority: 'high' | 'medium' | 'low';
     completed: boolean;
   }

   const tasks$ = from([
     { id: 1, priority: 'high' as const, completed: false },
     { id: 2, priority: 'high' as const, completed: false },
     { id: 3, priority: 'medium' as const, completed: false },
     { id: 4, priority: 'low' as const, completed: false },
   ] as Task[]);

   // Traiter uniquement tant que la priorité est high
   tasks$.pipe(
     takeWhile(task => task.priority === 'high')
   ).subscribe(task => {
     console.log(`Traitement de la tâche ${task.id}`);
   });
   // Sortie: Traitement de la tâche 1, Traitement de la tâche 2
   ```

3. **Traitement de pagination**
   ```ts
   import { range } from 'rxjs';
   import { takeWhile, map } from 'rxjs';

   interface Page {
     pageNumber: number;
     hasMore: boolean;
   }

   const pages$ = range(1, 10).pipe(
     map(pageNum => ({
       pageNumber: pageNum,
       hasMore: pageNum < 5
     } as Page))
   );

   // Charger les pages tant que hasMore est true
   pages$.pipe(
     takeWhile(page => page.hasMore, true) // inclusive: true
   ).subscribe(page => {
     console.log(`Chargement de la page ${page.pageNumber}`);
   });
   // Sortie: Chargement des pages 1 à 5
   ```


## 🧠 Exemple de code pratique (limite de compteur)

Exemple de compteur qui continue jusqu'à atteindre une condition spécifique.

```ts
import { fromEvent, interval } from 'rxjs';
import { takeWhile, scan, switchMap } from 'rxjs';

// Création des éléments UI
const container = document.createElement('div');
document.body.appendChild(container);

const startButton = document.createElement('button');
startButton.textContent = 'Démarrer le comptage';
container.appendChild(startButton);

const counter = document.createElement('div');
counter.style.fontSize = '24px';
counter.style.marginTop = '10px';
counter.textContent = 'Compteur: 0';
container.appendChild(counter);

const message = document.createElement('div');
message.style.marginTop = '5px';
message.style.color = 'gray';
message.textContent = 'Continue à compter tant que inférieur à 10';
container.appendChild(message);

// Démarrer le comptage au clic
fromEvent(startButton, 'click').pipe(
  switchMap(() =>
    interval(500).pipe(
      scan(count => count + 1, 0),
      takeWhile(count => count < 10)
    )
  )
).subscribe({
  next: (count) => {
    counter.textContent = `Compteur: ${count}`;
    startButton.disabled = true;
  },
  complete: () => {
    message.textContent = 'Terminé car 10 a été atteint!';
    message.style.color = 'green';
    startButton.disabled = false;
  }
});
```

Ce code compte de 0 à 9 et se termine automatiquement juste avant d'atteindre 10.


## 🎯 Comparaison avec skipWhile

`takeWhile` et `skipWhile` ont des comportements opposés.

```ts
import { range } from 'rxjs';
import { takeWhile, skipWhile } from 'rxjs';

const numbers$ = range(0, 10);

// takeWhile: récupère tant que la condition est satisfaite
numbers$.pipe(
  takeWhile(n => n < 5)
).subscribe(console.log);
// Sortie: 0, 1, 2, 3, 4

// skipWhile: ignore tant que la condition est satisfaite
numbers$.pipe(
  skipWhile(n => n < 5)
).subscribe(console.log);
// Sortie: 5, 6, 7, 8, 9
```

| Opérateur | Comportement | Moment de fin |
|---|---|---|
| `takeWhile(predicate)` | **Récupère** tant que la condition est satisfaite | Quand la condition devient `false` |
| `skipWhile(predicate)` | **Ignore** tant que la condition est satisfaite | Fin du flux original |


## 📋 Utilisation type-safe

Exemple d'implémentation type-safe avec les génériques TypeScript.

```ts
import { Observable, from } from 'rxjs';
import { takeWhile } from 'rxjs';

interface SensorReading {
  timestamp: Date;
  value: number;
  unit: string;
  status: 'normal' | 'warning' | 'critical';
}

function getReadingsUntilWarning(
  readings$: Observable<SensorReading>
): Observable<SensorReading> {
  return readings$.pipe(
    takeWhile(reading => reading.status === 'normal')
  );
}

// Exemple d'utilisation
const readings$ = from([
  { timestamp: new Date(), value: 25, unit: '°C', status: 'normal' as const },
  { timestamp: new Date(), value: 28, unit: '°C', status: 'normal' as const },
  { timestamp: new Date(), value: 32, unit: '°C', status: 'warning' as const },
  { timestamp: new Date(), value: 35, unit: '°C', status: 'critical' as const },
] as SensorReading[]);

getReadingsUntilWarning(readings$).subscribe(reading => {
  console.log(`${reading.value}${reading.unit} - ${reading.status}`);
});
// Sortie:
// 25°C - normal
// 28°C - normal
```


## 🔄 Différence entre takeWhile et filter

`takeWhile` se termine, contrairement à `filter`.

```ts
import { range } from 'rxjs';
import { takeWhile, filter } from 'rxjs';

const numbers$ = range(0, 10);

// filter: seules les valeurs correspondantes passent (le flux continue)
numbers$.pipe(
  filter(n => n < 5)
).subscribe({
  next: console.log,
  complete: () => console.log('filter terminé')
});
// Sortie: 0, 1, 2, 3, 4, filter terminé

// takeWhile: uniquement tant que la condition est satisfaite (termine quand false)
numbers$.pipe(
  takeWhile(n => n < 5)
).subscribe({
  next: console.log,
  complete: () => console.log('takeWhile terminé')
});
// Sortie: 0, 1, 2, 3, 4, takeWhile terminé
```

| Opérateur | Comportement | Fin du flux |
|---|---|---|
| `filter(predicate)` | Seules les valeurs correspondantes passent | À la fin du flux original |
| `takeWhile(predicate)` | Récupère tant que la condition est satisfaite | Quand la condition devient `false` |


## ⚠️ Erreurs courantes

> [!NOTE]
> Si la condition est `false` dès le départ avec `takeWhile`, rien n'est émis et le flux se termine. Assurez-vous que la condition est correctement configurée.

### Incorrect: la condition est false dès le départ

```ts
import { range } from 'rxjs';
import { takeWhile } from 'rxjs';

// ❌ Mauvais exemple: la condition est false dès la première valeur
range(5, 10).pipe(
  takeWhile(n => n < 5)
).subscribe(console.log);
// Rien n'est émis (la première valeur 5 rend la condition false)
```

### Correct: vérifier la condition

```ts
import { range } from 'rxjs';
import { takeWhile } from 'rxjs';

// ✅ Bon exemple: configurer correctement la condition
range(0, 10).pipe(
  takeWhile(n => n < 5)
).subscribe(console.log);
// Sortie: 0, 1, 2, 3, 4
```


## 🎓 Résumé

### Quand utiliser takeWhile
- ✅ Lorsque vous voulez contrôler le flux avec des conditions dynamiques
- ✅ Lorsque vous voulez récupérer des données jusqu'à un seuil
- ✅ Lorsque vous voulez traiter uniquement pendant qu'un état spécifique persiste
- ✅ Lorsque vous avez besoin d'une fin anticipée basée sur une condition

### Quand utiliser take
- ✅ Lorsque le nombre d'éléments à récupérer est déterminé
- ✅ Lorsque vous avez besoin d'une simple limitation par nombre

### Quand utiliser filter
- ✅ Lorsque vous voulez extraire uniquement les valeurs correspondantes de tout le flux
- ✅ Lorsque vous ne voulez pas terminer le flux

### Points d'attention
- ⚠️ Si la condition est `false` dès le départ, rien n'est émis et le flux se termine
- ⚠️ Par défaut, la valeur qui rend la condition `false` n'est pas émise (utilisez `inclusive: true` pour l'inclure)
- ⚠️ Avec un flux infini, si la condition est toujours `true`, le flux continue indéfiniment


## 🚀 Prochaines étapes

- **[take](./take)** - Apprendre à récupérer les N premières valeurs
- **[takeLast](./takeLast)** - Apprendre à récupérer les N dernières valeurs
- **[takeUntil](../utility/takeUntil)** - Apprendre à récupérer jusqu'à ce qu'un autre Observable émette
- **[filter](./filter)** - Apprendre le filtrage basé sur les conditions
- **[Exemples pratiques d'opérateurs de filtrage](./practical-use-cases)** - Apprendre des cas d'utilisation réels

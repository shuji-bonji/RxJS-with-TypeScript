---
description: "skipLast est un opérateur de filtrage RxJS qui ignore les N dernières valeurs d'un flux Observable et n'émet que les valeurs précédentes."
---

# skipLast - Ignorer les N dernières valeurs

L'opérateur `skipLast` **ignore les N dernières valeurs** émises par l'Observable source et n'émet que les valeurs précédentes. Il conserve les N dernières valeurs dans un tampon jusqu'à la fin du flux, et émet les autres.

## 🔰 Syntaxe de base et utilisation

```ts
import { range } from 'rxjs';
import { skipLast } from 'rxjs';

const numbers$ = range(0, 10); // 0 à 9

numbers$.pipe(
  skipLast(3)
).subscribe(console.log);
// Sortie: 0, 1, 2, 3, 4, 5, 6
// (7, 8, 9 sont ignorés)
```

**Flux d'opération** :
1. Le flux émet 0, 1, 2, ...
2. Les 3 dernières valeurs (7, 8, 9) sont conservées dans le tampon
3. Les valeurs dépassant la taille du tampon (0 à 6) sont émises
4. À la fin du flux, les valeurs du tampon (7, 8, 9) sont ignorées

[🌐 Documentation officielle RxJS - `skipLast`](https://rxjs.dev/api/operators/skipLast)

## 💡 Patterns d'utilisation typiques

- **Exclusion des données récentes** : exclure les données récentes non confirmées
- **Traitement par lots** : exclure les données non confirmées avant la fin du traitement
- **Validation des données** : lorsque la validation par les valeurs suivantes est nécessaire
- **Traitement des données à confirmation différée** : lorsque les N dernières ne sont pas confirmées

## 🧠 Exemple de code pratique 1 : Pipeline de traitement de données

Exemple d'ignorance des données non confirmées lors du traitement de données.

```ts
import { from, interval } from 'rxjs';
import { skipLast, map, take, concatMap, delay } from 'rxjs';

// Création de l'UI
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'Pipeline de traitement de données';
container.appendChild(title);

const description = document.createElement('div');
description.style.marginBottom = '10px';
description.style.color = '#666';
description.textContent = 'Ignore les 2 derniers éléments (données non confirmées) et traite le reste';
container.appendChild(description);

const output = document.createElement('div');
output.style.border = '1px solid #ccc';
output.style.padding = '10px';
output.style.maxHeight = '200px';
output.style.overflow = 'auto';
container.appendChild(output);

interface DataPoint {
  id: number;
  value: number;
  status: 'processing' | 'confirmed' | 'skipped';
}

// Flux de données (10 éléments)
const data: DataPoint[] = Array.from({ length: 10 }, (_, i) => ({
  id: i,
  value: Math.floor(Math.random() * 100),
  status: 'processing' as const
}));

// Émet les données toutes les 0.5 secondes
from(data).pipe(
  concatMap(item => interval(500).pipe(
    take(1),
    map(() => item)
  )),
  skipLast(2) // Ignore les 2 derniers
).subscribe({
  next: item => {
    const div = document.createElement('div');
    div.style.padding = '5px';
    div.style.marginBottom = '5px';
    div.style.backgroundColor = '#e8f5e9';
    div.style.border = '1px solid #4CAF50';
    div.innerHTML = `
      <strong>✅ Confirmé</strong>
      ID: ${item.id} |
      Valeur: ${item.value}
    `;
    output.appendChild(div);
  },
  complete: () => {
    // Affiche les éléments ignorés
    const skippedItems = data.slice(-2);
    skippedItems.forEach(item => {
      const div = document.createElement('div');
      div.style.padding = '5px';
      div.style.marginBottom = '5px';
      div.style.backgroundColor = '#ffebee';
      div.style.border = '1px solid #f44336';
      div.innerHTML = `
        <strong>⏭️ Ignoré</strong>
        ID: ${item.id} |
        Valeur: ${item.value} |
        (Données non confirmées)
      `;
      output.appendChild(div);
    });

    const summary = document.createElement('div');
    summary.style.marginTop = '10px';
    summary.style.padding = '10px';
    summary.style.backgroundColor = '#e3f2fd';
    summary.textContent = `Traitement terminé: ${data.length - 2} confirmés, 2 ignorés`;
    output.appendChild(summary);
  }
});
```

- Les données sont traitées séquentiellement, mais les 2 dernières sont traitées comme non confirmées et ignorées.
- Après la fin, les éléments ignorés sont également affichés.

## 🎯 Exemple de code pratique 2 : Filtrage des logs

Exemple d'ignorance des logs récents non confirmés d'un flux de logs.

```ts
import { interval } from 'rxjs';
import { skipLast, map, take } from 'rxjs';

// Création de l'UI
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'Surveillance des logs';
container.appendChild(title);

const info = document.createElement('div');
info.style.marginBottom = '10px';
info.textContent = 'Les 3 derniers logs sont ignorés comme en attente de confirmation';
info.style.color = '#666';
container.appendChild(info);

const confirmedLogs = document.createElement('div');
confirmedLogs.innerHTML = '<strong>📋 Logs confirmés :</strong>';
confirmedLogs.style.marginBottom = '10px';
container.appendChild(confirmedLogs);

const confirmedList = document.createElement('div');
confirmedList.style.border = '1px solid #4CAF50';
confirmedList.style.padding = '10px';
confirmedList.style.backgroundColor = '#f1f8e9';
confirmedList.style.minHeight = '100px';
container.appendChild(confirmedList);

const pendingLogs = document.createElement('div');
pendingLogs.innerHTML = '<strong>⏳ Logs en attente (ignorés) :</strong>';
pendingLogs.style.marginTop = '10px';
pendingLogs.style.marginBottom = '10px';
container.appendChild(pendingLogs);

const pendingList = document.createElement('div');
pendingList.style.border = '1px solid #FF9800';
pendingList.style.padding = '10px';
pendingList.style.backgroundColor = '#fff3e0';
pendingList.style.minHeight = '60px';
container.appendChild(pendingList);

interface LogEntry {
  id: number;
  timestamp: Date;
  level: 'info' | 'warn' | 'error';
  message: string;
}

// Génère des logs (12 au total, 1 par seconde)
const logs$ = interval(1000).pipe(
  take(12),
  map(i => {
    const levels: ('info' | 'warn' | 'error')[] = ['info', 'warn', 'error'];
    const messages = [
      'Connexion utilisateur',
      'Début de récupération des données',
      'Mise à jour du cache',
      'Erreur de connexion',
      'Nouvelle tentative en cours',
      'Traitement des données terminé'
    ];
    return {
      id: i,
      timestamp: new Date(),
      level: levels[Math.floor(Math.random() * levels.length)],
      message: messages[Math.floor(Math.random() * messages.length)]
    } as LogEntry;
  })
);

const allLogs: LogEntry[] = [];

// Enregistre tous les logs (pour vérification)
logs$.subscribe(log => {
  allLogs.push(log);
});

// Ignore les 3 derniers et affiche les logs confirmés
logs$.pipe(
  skipLast(3)
).subscribe({
  next: log => {
    const logDiv = document.createElement('div');
    logDiv.style.padding = '3px';
    logDiv.style.marginBottom = '3px';
    const icon = log.level === 'error' ? '❌' : log.level === 'warn' ? '⚠️' : 'ℹ️';
    logDiv.textContent = `${icon} [${log.id}] ${log.timestamp.toLocaleTimeString()} - ${log.message}`;
    confirmedList.appendChild(logDiv);
  },
  complete: () => {
    // Affiche les 3 derniers (logs ignorés)
    const skippedLogs = allLogs.slice(-3);
    skippedLogs.forEach(log => {
      const logDiv = document.createElement('div');
      logDiv.style.padding = '3px';
      logDiv.style.marginBottom = '3px';
      const icon = log.level === 'error' ? '❌' : log.level === 'warn' ? '⚠️' : 'ℹ️';
      logDiv.textContent = `${icon} [${log.id}] ${log.timestamp.toLocaleTimeString()} - ${log.message}`;
      pendingList.appendChild(logDiv);
    });
  }
});
```

- Les logs sont ajoutés séquentiellement, mais les 3 derniers sont ignorés comme en attente de confirmation.
- Après la fin, les logs ignorés sont également affichés.

## 🆚 Comparaison avec des opérateurs similaires

### skipLast vs takeLast vs skip

```ts
import { range } from 'rxjs';
import { skipLast, takeLast, skip } from 'rxjs';

const numbers$ = range(0, 10); // 0 à 9

// skipLast: ignore les N derniers
numbers$.pipe(
  skipLast(3)
).subscribe(console.log);
// Sortie: 0, 1, 2, 3, 4, 5, 6

// takeLast: récupère uniquement les N derniers
numbers$.pipe(
  takeLast(3)
).subscribe(console.log);
// Sortie: 7, 8, 9

// skip: ignore les N premiers
numbers$.pipe(
  skip(3)
).subscribe(console.log);
// Sortie: 3, 4, 5, 6, 7, 8, 9
```

| Opérateur | Position ignorée | Moment d'émission | Attente de fin |
|:---|:---|:---|:---|
| `skipLast(n)` | Les n derniers | Émission quand le tampon est plein | Nécessaire |
| `takeLast(n)` | Tout sauf les n derniers | Émission groupée après fin | Nécessaire |
| `skip(n)` | Les n premiers | Émission immédiate | Non nécessaire |

**Différence visuelle** :

```
Entrée: 0, 1, 2, 3, 4, 5, 6, 7, 8, 9

skipLast(3): 0, 1, 2, 3, 4, 5, 6 | [7, 8, 9 ignorés]
                                   ^les 3 derniers

takeLast(3): [0~6 ignorés] | 7, 8, 9
                             ^uniquement les 3 derniers

skip(3): [0, 1, 2 ignorés] | 3, 4, 5, 6, 7, 8, 9
          ^les 3 premiers
```

## ⚠️ Points d'attention

### 1. Comportement avec les flux infinis

`skipLast` ne peut pas déterminer les N derniers tant que le flux ne se termine pas, donc il ne fonctionne pas comme prévu avec les flux infinis.

```ts
import { interval } from 'rxjs';
import { skipLast } from 'rxjs';

// ❌ Mauvais exemple: utiliser skipLast avec un flux infini
interval(1000).pipe(
  skipLast(3)
).subscribe(console.log);
// Sortie: 0 (après 3s), 1 (après 4s), 2 (après 5s), ...
// Toutes les valeurs sont émises avec un délai de N
// Les 3 derniers restent dans le tampon indéfiniment
```

Avec les flux infinis, les N derniers ne peuvent pas être déterminés, donc toutes les valeurs sont émises avec un délai de N. Les vrais « N derniers » n'existent pas, donc l'objectif de `skipLast` ne peut pas être atteint.

**Solution** : créer un flux fini avec `take`

```ts
import { interval } from 'rxjs';
import { take, skipLast } from 'rxjs';

// ✅ Bon exemple: créer un flux fini puis utiliser skipLast
interval(1000).pipe(
  take(10),      // Terminer après les 10 premiers
  skipLast(3)    // Ignorer les 3 derniers
).subscribe(console.log);
// Sortie: 0, 1, 2, 3, 4, 5, 6
// (7, 8, 9 sont ignorés)
```

### 2. Attention à la taille du tampon

`skipLast(n)` conserve toujours n valeurs dans le tampon.

```ts
import { range } from 'rxjs';
import { skipLast } from 'rxjs';

// ⚠️ 1000 éléments dans le tampon
range(0, 1000000).pipe(
  skipLast(1000)
).subscribe(console.log);
```

### 3. Délai d'émission

`skipLast(n)` n'émet rien tant que le tampon n'a pas n éléments.

```ts
import { interval } from 'rxjs';
import { take, skipLast, tap } from 'rxjs';

interval(1000).pipe(
  take(5),
  tap(val => console.log('Entrée:', val)),
  skipLast(2)
).subscribe(val => console.log('Sortie:', val));
// Entrée: 0
// Entrée: 1
// Entrée: 2
// Sortie: 0  ← émission commence quand le tampon a 2 éléments
// Entrée: 3
// Sortie: 1
// Entrée: 4
// Sortie: 2
// Terminé (3, 4 sont ignorés)
```

### 4. Comportement de skipLast(0)

`skipLast(0)` n'ignore rien.

```ts
import { range } from 'rxjs';
import { skipLast } from 'rxjs';

range(0, 5).pipe(
  skipLast(0)
).subscribe(console.log);
// Sortie: 0, 1, 2, 3, 4 (tout est émis)
```

## 💡 Patterns de combinaison pratiques

### Pattern 1 : Récupérer uniquement la partie intermédiaire

Ignorer le début et la fin et récupérer uniquement la partie intermédiaire

```ts
import { range } from 'rxjs';
import { skip, skipLast } from 'rxjs';

range(0, 10).pipe(
  skip(2),      // Ignorer les 2 premiers
  skipLast(2)   // Ignorer les 2 derniers
).subscribe(console.log);
// Sortie: 2, 3, 4, 5, 6, 7
```

### Pattern 2 : Validation des données

Lorsque la validation par les valeurs suivantes est nécessaire

```ts
import { from } from 'rxjs';
import { skipLast, map } from 'rxjs';

interface Transaction {
  id: number;
  amount: number;
  pending: boolean;
}

const transactions$ = from([
  { id: 1, amount: 100, pending: false },
  { id: 2, amount: 200, pending: false },
  { id: 3, amount: 150, pending: false },
  { id: 4, amount: 300, pending: true },  // Non confirmé
  { id: 5, amount: 250, pending: true }   // Non confirmé
]);

// Ignorer les transactions non confirmées (les 2 dernières)
transactions$.pipe(
  skipLast(2)
).subscribe(tx => {
  console.log(`Confirmé: ID ${tx.id}, Montant ${tx.amount}€`);
});
// Sortie:
// Confirmé: ID 1, Montant 100€
// Confirmé: ID 2, Montant 200€
// Confirmé: ID 3, Montant 150€
```

### Pattern 3 : Traitement par fenêtre

Traitement par fenêtre avec les N derniers exclus

```ts
import { range } from 'rxjs';
import { skipLast, bufferCount } from 'rxjs';

range(0, 10).pipe(
  skipLast(2),      // Ignorer les 2 derniers
  bufferCount(3, 1) // Fenêtres de 3
).subscribe(window => {
  console.log('Fenêtre:', window);
});
// Sortie:
// Fenêtre: [0, 1, 2]
// Fenêtre: [1, 2, 3]
// Fenêtre: [2, 3, 4]
// ...
```

## 📚 Opérateurs associés

- **[skip](./skip)** - Ignorer les N premières valeurs
- **[takeLast](./takeLast)** - Récupérer uniquement les N dernières valeurs
- **[take](./take)** - Récupérer uniquement les N premières valeurs
- **[skipUntil](./skipUntil)** - Ignorer jusqu'à ce qu'un autre Observable émette
- **[skipWhile](./skipWhile)** - Ignorer tant que la condition est satisfaite

## Résumé

L'opérateur `skipLast` ignore les N dernières valeurs d'un flux.

- ✅ Idéal lorsque les N dernières données ne sont pas nécessaires
- ✅ Pratique pour exclure les données non confirmées
- ✅ Taille du tampon limitée à N (bonne efficacité mémoire)
- ✅ Nécessite la fin du flux
- ⚠️ Ne peut pas être utilisé avec les flux infinis
- ⚠️ Pas d'émission tant que le tampon n'a pas N éléments
- ⚠️ Souvent nécessaire de combiner avec `take` pour créer un flux fini

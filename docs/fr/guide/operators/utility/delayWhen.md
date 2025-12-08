---
description: "L'opérateur delayWhen contrôle dynamiquement le délai de chaque valeur avec des Observables individuels. Il permet un traitement flexible des délais basé sur des conditions, un backoff exponentiel pour les tentatives, l'attente des actions utilisateur, l'ajustement des intervalles d'appel API, et d'autres modèles pratiques expliqués avec des exemples de code TypeScript."
---

# delayWhen - Contrôle dynamique des délais

L'opérateur `delayWhen` **détermine le temps de délai pour chaque valeur dynamiquement en utilisant des Observables individuels**. Alors que l'opérateur `delay` fournit un délai fixe, `delayWhen` peut appliquer différents délais à chaque valeur.

## 🔰 Syntaxe et comportement de base

Spécifiez une fonction qui retourne un Observable pour déterminer le délai pour chaque valeur.

```ts
import { of, timer } from 'rxjs';
import { delayWhen } from 'rxjs';

of('A', 'B', 'C')
  .pipe(
    delayWhen(value => {
      const delayTime = value === 'B' ? 2000 : 1000;
      return timer(delayTime);
    })
  )
  .subscribe(console.log);
// Sortie :
// A (après 1 seconde)
// C (après 1 seconde)
// B (après 2 secondes)
```

Dans cet exemple, seule la valeur `'B'` a un délai de 2 secondes appliqué, tandis que les autres ont un délai de 1 seconde.

[🌐 Documentation officielle RxJS - delayWhen](https://rxjs.dev/api/index/function/delayWhen)

## 💡 Cas d'utilisation typiques

- **Délai basé sur la valeur** : Modification du temps de délai selon la priorité ou le type
- **Délai par événement externe** : Attente des actions utilisateur ou de l'achèvement d'autres flux
- **Délai conditionnel** : Retarder uniquement des valeurs spécifiques
- **Contrôle du timing asynchrone** : Attente des réponses API ou de la disponibilité des données

## 🧪 Exemple de code pratique 1 : Délai basé sur la priorité

Exemple contrôlant le timing de traitement selon la priorité des tâches.

```ts
import { from, timer } from 'rxjs';
import { delayWhen } from 'rxjs';

// Création de l'interface utilisateur
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'delayWhen - Délai basé sur la priorité';
container.appendChild(title);

const output = document.createElement('div');
output.style.border = '1px solid #ccc';
output.style.padding = '10px';
output.style.maxHeight = '300px';
output.style.overflow = 'auto';
container.appendChild(output);

interface Task {
  id: number;
  name: string;
  priority: 'high' | 'medium' | 'low';
}

const tasks: Task[] = [
  { id: 1, name: 'Tâche A', priority: 'low' },
  { id: 2, name: 'Tâche B', priority: 'high' },
  { id: 3, name: 'Tâche C', priority: 'medium' },
  { id: 4, name: 'Tâche D', priority: 'high' },
  { id: 5, name: 'Tâche E', priority: 'low' }
];

function addLog(message: string, color: string) {
  const logItem = document.createElement('div');
  logItem.style.padding = '5px';
  logItem.style.marginBottom = '3px';
  logItem.style.backgroundColor = color;

  const now = new Date();
  const time = now.toLocaleTimeString('fr-FR', { hour12: false }) +
    '.' + now.getMilliseconds().toString().padStart(3, '0');

  logItem.textContent = `[${time}] ${message}`;
  output.appendChild(logItem);
}

addLog('Début du traitement', '#e3f2fd');

from(tasks)
  .pipe(
    delayWhen(task => {
      // Définir le temps de délai selon la priorité
      let delayTime: number;
      switch (task.priority) {
        case 'high':
          delayTime = 500;  // Haute priorité : 0,5 seconde
          break;
        case 'medium':
          delayTime = 1500; // Priorité moyenne : 1,5 seconde
          break;
        case 'low':
          delayTime = 3000; // Basse priorité : 3 secondes
          break;
      }
      return timer(delayTime);
    })
  )
  .subscribe({
    next: task => {
      const colors = {
        high: '#c8e6c9',
        medium: '#fff9c4',
        low: '#ffccbc'
      };
      addLog(
        `${task.name} (priorité : ${task.priority}) traité`,
        colors[task.priority]
      );
    },
    complete: () => {
      addLog('Toutes les tâches sont terminées', '#e3f2fd');
    }
  });
```

- Les tâches à haute priorité sont traitées après 0,5 seconde
- Priorité moyenne après 1,5 seconde, basse priorité après 3 secondes
- Réalise un ordre de traitement selon l'importance des tâches

## 🧪 Exemple de code pratique 2 : Délai par événement externe

Exemple attendant les clics de l'utilisateur avant d'émettre les valeurs.

```ts
import { of, fromEvent } from 'rxjs';
import { delayWhen, take, tap } from 'rxjs';

// Création de l'interface utilisateur
const container2 = document.createElement('div');
container2.style.marginTop = '20px';
document.body.appendChild(container2);

const title2 = document.createElement('h3');
title2.textContent = 'delayWhen - Attente de clic';
container2.appendChild(title2);

const button = document.createElement('button');
button.textContent = 'Cliquez pour afficher la valeur suivante';
button.style.marginBottom = '10px';
container2.appendChild(button);

const output2 = document.createElement('div');
output2.style.border = '1px solid #ccc';
output2.style.padding = '10px';
output2.style.minHeight = '100px';
container2.appendChild(output2);

function addLog2(message: string) {
  const logItem = document.createElement('div');
  logItem.style.padding = '5px';
  logItem.style.marginBottom = '3px';
  logItem.textContent = message;
  output2.appendChild(logItem);
}

let clickCount = 0;

of('Message 1', 'Message 2', 'Message 3')
  .pipe(
    tap(msg => {
      addLog2(`En attente : ${msg} (cliquez sur le bouton)`);
      button.textContent = `Cliquez pour afficher « ${msg} »`;
    }),
    delayWhen(() => {
      // Délai jusqu'à ce qu'un événement de clic se produise
      return fromEvent(button, 'click').pipe(take(1));
    })
  )
  .subscribe({
    next: msg => {
      clickCount++;
      addLog2(`✅ Affiché : ${msg}`);
      if (clickCount < 3) {
        button.disabled = false;
      } else {
        button.textContent = 'Terminé';
        button.disabled = true;
      }
    },
    complete: () => {
      addLog2('--- Tous les messages ont été affichés ---');
    }
  });
```

- Chaque valeur est émise après l'attente du clic de l'utilisateur
- Contrôle de délai déclenché par des événements externes possible
- Applicable au traitement de séquences interactives

## 🆚 Comparaison avec delay

```ts
import { of, timer } from 'rxjs';
import { delay, delayWhen } from 'rxjs';

// delay - Délai à temps fixe
of(1, 2, 3)
  .pipe(delay(1000))
  .subscribe(console.log);
// Toutes les valeurs sont retardées de 1 seconde

// delayWhen - Délai différent par valeur
of(1, 2, 3)
  .pipe(
    delayWhen(value => timer(value * 1000))
  )
  .subscribe(console.log);
// 1 après 1 seconde, 2 après 2 secondes, 3 après 3 secondes
```

| Opérateur | Contrôle du délai | Cas d'utilisation |
|:---|:---|:---|
| `delay` | Temps fixe | Délai uniforme simple |
| `delayWhen` | Dynamique (par valeur) | Délai conditionnel, attente d'événement externe |

## ⚠️ Notes importantes

### 1. L'Observable de délai est généré à chaque fois

```ts
// ❌ Mauvais exemple : Réutilisation de la même instance Observable
const delayObs$ = timer(1000);
source$.pipe(
  delayWhen(() => delayObs$)  // Ne fonctionne pas à partir de la 2e fois
).subscribe();

// ✅ Bon exemple : Génération d'un nouvel Observable à chaque fois
source$.pipe(
  delayWhen(() => timer(1000))
).subscribe();
```

### 2. Si l'Observable de délai ne se termine jamais

```ts
import { of, NEVER } from 'rxjs';
import { delayWhen } from 'rxjs';

// ❌ Mauvais exemple : Retourner NEVER cause un délai éternel
of(1, 2, 3)
  .pipe(
    delayWhen(() => NEVER)  // Les valeurs ne sont jamais émises
  )
  .subscribe(console.log);
// Aucune sortie
```

L'Observable de délai doit émettre une valeur ou se terminer.

### 3. Gestion des erreurs

Si une erreur survient dans l'Observable de délai, le flux entier sera en erreur.

```ts
import { of, throwError, timer, delayWhen } from 'rxjs';

of(1, 2, 3)
  .pipe(
    delayWhen(value => {
      if (value === 2) {
        return throwError(() => new Error('Erreur de délai'));
      }
      return timer(1000);
    })
  )
  .subscribe({
    next: console.log,
    error: err => console.error('Erreur :', err.message)
  });
// Sortie : 1
// Erreur : Erreur de délai
```

## 📚 Opérateurs associés

- **[delay](./delay)** - Délai à temps fixe
- **[debounceTime](../filtering/debounceTime)** - Délai après arrêt de l'entrée
- **[throttleTime](../filtering/throttleTime)** - Passage de valeur à intervalles réguliers
- **[timeout](./timeout)** - Contrôle du délai d'expiration

## ✅ Résumé

L'opérateur `delayWhen` contrôle dynamiquement le timing de délai de chaque valeur.

- ✅ Possibilité d'appliquer différents délais par valeur
- ✅ Contrôle de délai par événements externes ou Observables
- ✅ Ajustement du timing de traitement selon la priorité ou le type
- ⚠️ L'Observable de délai doit être généré à chaque fois
- ⚠️ L'Observable de délai doit se terminer ou émettre une valeur
- ⚠️ Attention à la gestion des erreurs

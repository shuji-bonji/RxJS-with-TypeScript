---
description: "L'opérateur repeat ré-exécute le flux entier un nombre spécifié de fois après que l'Observable source se termine normalement. Il peut être utilisé pour le traitement de polling périodique, les animations répétées et les situations où un contrôle différent de retry est nécessaire."
---

# repeat - Répétition de flux

L'opérateur `repeat` **ré-exécute le flux entier un nombre spécifié de fois après que l'Observable source se termine normalement**.
Il est utile pour le traitement de polling, les animations répétées et les contrôles différents de retry.

## 🔰 Syntaxe et comportement de base

L'utilisation la plus simple est de répéter une séquence de valeurs un certain nombre de fois.

```ts
import { of } from 'rxjs';
import { repeat } from 'rxjs';

of('A', 'B')
  .pipe(
    repeat(2) // Répéter le flux entier 2 fois (sortie 2 fois au total)
  )
  .subscribe(console.log);
// Sortie :
// A
// B
// A
// B
```

[🌐 Documentation officielle RxJS - repeat](https://rxjs.dev/api/index/function/repeat)

## 💡 Cas d'utilisation typiques

Par exemple, il est utilisé pour un simple traitement de polling ou des animations d'affichage répétées.

```ts
import { of } from 'rxjs';
import { tap, delay, repeat } from 'rxjs';

of('✅ Récupération des données réussie')
  .pipe(
    tap(() => console.log('Requête démarrée')),
    delay(1000),
    repeat(3) // Répéter 3 fois
  )
  .subscribe(console.log);
// Sortie :
// Requête démarrée
// ✅ Récupération des données réussie
// main.ts:6 Requête démarrée
// ✅ Récupération des données réussie
// main.ts:6 Requête démarrée
// ✅ Récupération des données réussie
```

Dans cet exemple, "Requête → Récupération de données" est répété 3 fois à 1 seconde d'intervalle.

## 🧪 Exemple de code pratique (avec interface utilisateur)

```ts
import { of } from 'rxjs';
import { repeat, tap } from 'rxjs';

// Zone d'affichage de la sortie
const repeatOutput = document.createElement('div');
repeatOutput.innerHTML = '<h3>Exemple de repeat :</h3>';
document.body.appendChild(repeatOutput);

// Affichage du nombre de répétitions
let repeatCount = 0;
const repeatCountDisplay = document.createElement('div');
repeatCountDisplay.textContent = `Compte de répétition : ${repeatCount}`;
repeatCountDisplay.style.fontWeight = 'bold';
repeatOutput.appendChild(repeatCountDisplay);

// Zone de sortie des valeurs
const valuesOutput = document.createElement('div');
valuesOutput.style.marginTop = '10px';
valuesOutput.style.padding = '10px';
valuesOutput.style.border = '1px solid #ddd';
valuesOutput.style.maxHeight = '200px';
valuesOutput.style.overflowY = 'auto';
repeatOutput.appendChild(valuesOutput);

// Répétition de séquence
of('A', 'B', 'C')
  .pipe(
    tap(() => {
      repeatCount++;
      repeatCountDisplay.textContent = `Compte de répétition : ${repeatCount}`;
    }),
    repeat(3)
  )
  .subscribe((val) => {
    const valueItem = document.createElement('div');
    valueItem.textContent = `Valeur : ${val} (répétition ${repeatCount})`;
    valuesOutput.appendChild(valueItem);
  });

```

## ✅ Résumé

- `repeat` **ré-exécute l'Observable entier après qu'il se soit terminé normalement**
- Contrairement à `retry`, **il ne réexécute pas en cas d'erreur**
- Peut également être utilisé pour le traitement de polling ou le **clignotement de placeholder** et d'autres animations répétées

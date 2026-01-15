---
description: "L'opérateur exhaustMap ignore les nouvelles entrées pendant le traitement de l'Observable actuel. Efficace pour éviter les clics en double sur les boutons de soumission de formulaires ou les requêtes API en double."
---

# exhaustMap - Ignorer Pendant Actif

L'opérateur `exhaustMap` **ignore les nouvelles entrées** jusqu'à ce que l'Observable actuel soit terminé.
Parfait pour éviter les clics en double ou les soumissions de requêtes en double.

## 🔰 Syntaxe de base et utilisation

```ts
import { fromEvent, of } from 'rxjs';
import { exhaustMap, delay } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

clicks$
  .pipe(exhaustMap(() => of('Requête terminée').pipe(delay(1000))))
  .subscribe(console.log);

// Sortie :
// (seul le premier clic affiche « Requête terminée » après 1 seconde)
```

- Les entrées suivantes sont ignorées jusqu'à ce que la requête actuelle soit terminée.

[🌐 Documentation officielle RxJS - exhaustMap](https://rxjs.dev/api/operators/exhaustMap)

## 💡 Modes d'utilisation typiques

- Empêcher les clics en double sur les boutons de soumission de formulaires
- Empêcher les requêtes en double (en particulier pour le traitement des connexions/paiements)
- Contrôle d'affichage unique pour les fenêtres modales ou les boîtes de dialogue

## 🧠 Exemple de code pratique (avec interface utilisateur)

Lorsque le bouton de soumission est cliqué, le traitement de la soumission commence.
**Même si cliqué plusieurs fois pendant la soumission, les clics sont ignorés**, et la soumission suivante n'est pas acceptée tant que le premier processus n'est pas terminé.

```ts
import { fromEvent } from 'rxjs';
import { exhaustMap, delay } from 'rxjs';
import { ajax } from 'rxjs/ajax';

// Création du bouton
const submitButton = document.createElement('button');
submitButton.textContent = 'Soumettre';
document.body.appendChild(submitButton);

// Zone de sortie
const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// Traitement de la soumission
fromEvent(submitButton, 'click')
  .pipe(
    exhaustMap(() => {
      output.textContent = 'Soumission en cours...';
      return ajax
        .post('https://jsonplaceholder.typicode.com/posts', {
          title: 'foo',
          body: 'bar',
          userId: 1,
        })
        .pipe(delay(2000)); // Simulation d'un délai de soumission de 2 secondes
    })
  )
  .subscribe({
    next: (response) => {
      output.textContent = 'Soumission réussie !';
      console.log('Soumission réussie :', response);
    },
    error: (error) => {
      output.textContent = 'Erreur de soumission';
      console.error('Erreur de soumission :', error);
    },
  });
```

- Les autres clics pendant le traitement du bouton sont ignorés.
- « Soumission réussie ! » ou « Erreur de soumission » s'affiche après 2 secondes.

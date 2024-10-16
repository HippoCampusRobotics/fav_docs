Python 101
=============

This Python "Onramp" Course is in form of a Jupyter Notebook.
As soon as everything is set up, it reads similiar to a Matlab live script but instead of matlab code, Python code is embedded into the script.
Within the notebook you will find a short overview of basic Python synthax with examples and minor interactive tasks.


Get the template of the Jupyter Notebook
******************************************
For the next step you have to open your terminal.
To do so, you can always search through your applications for "terminal" or if you want to be cool, you can press :kbd:`Ctrl` + :kbd:`Alt` + :kbd:`T`.
Now simply **copy and paste** the following lines of code step by step:

.. note:: To Copy and paste within the terminal hit :kbd:`shift` + :kbd:`Ctrl` + :kbd:`C` / :kbd:`V`


.. code-block:: console

   $ cd && git clone https://github.com/HippoCampusRobotics/python-onramp.git

.. code-block:: console

   $ cd python-onramp

Create a virtual environment:

.. code-block:: console

   $ python3 -m venv venv

Source the environment:

.. code-block:: console
   
   $ source venv/bin/activate

Install the dependencies:

.. code-block:: console

   $ python3 -m pip install -r requirements.txt

Open the Notebook
#################

.. code-block:: console

   $ jupyter notebook

A new browser tab should open. 
Open the notebook by right clicking :file:`python101.py`

and selecting :menuselection:`Open with --> Notebook`

**Alternatively** you can open the notebook in VSCode.

Start VSCode and open the repository's directory. 
VSCode probablly suggests to install the Jupyter extension. If not, you can install it manually.

You can then open the file :file:`Python101.ipynb` and run the notebook.
